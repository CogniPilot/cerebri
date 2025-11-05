/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#endif

#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#ifdef CONFIG_FILE_SYSTEM_RPMSGFS
#include <synapse_rpmsg.h>
#endif

#define FS_RET_OK     FR_OK
#define MY_STACK_SIZE 2048
#define MY_PRIORITY   1
#define BUF_SIZE      512

extern struct ring_buf rb_sdcard;

#ifndef CONFIG_FILE_SYSTEM_RPMSGFS
static FATFS fat_fs;
#endif
static struct fs_mount_t mp = {
	.type = FS_FATFS,
};

LOG_MODULE_DECLARE(log_sdcard, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct fs_file_t file;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	size_t total_size_written;
};

static struct context g_ctx = {
	.file = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.total_size_written = 0,
};

static const char *disk_mount_pt = "/SD:";

static int mount_sd_card(void)
{
	struct fs_statvfs stat;
	int res;

	res = fs_statvfs(disk_mount_pt, &stat);
	if (res == -ENOENT) {

		mp.mnt_point = disk_mount_pt;

#ifndef CONFIG_FILE_SYSTEM_RPMSGFS
		mp.fs_data = &fat_fs;
#else
		mp.fs_data = "/cerebri";
		mp.type = FS_RPMSGFS;
		rpmsg_rpdev_wait();
#endif

		res = fs_mount(&mp);

		if (res == 0) {
			LOG_INF("Disk mounted.");
		} else {
			LOG_INF("Failed to mount disk - trying one more time");
			res = fs_mount(&mp);
			if (res != 0) {
				LOG_ERR("Error mounting disk.");
				return -1;
			}
		}

		do {
			res = fs_statvfs(disk_mount_pt, &stat);
			k_sleep(K_MSEC(1));
		} while (res >= 0 && stat.f_blocks == 0);
		/* Retry indefinitely until a mounted filesystem is available
		 * and has space to write to (i.e., f_blocks > 0).
		 */
	}

	if (res < 0) {
		return -1;
	}

	LOG_INF("Sector size %lu", stat.f_blocks);
	LOG_INF("Memory Size(MB) %u", (uint32_t)((stat.f_bsize * stat.f_blocks) >> 20));

	return 0;
}

static int log_sdcard_writer_init(struct context *ctx)
{
	int ret = 0;

	ret = mount_sd_card();
	if (ret < 0) {
		return ret;
	}

	fs_file_t_init(&ctx->file);

#ifdef CONFIG_FILE_SYSTEM_RPMSGFS
	/* Linux VFS is partially initialized: currently mounted read-only.
	 * Linux requires additional setup before the VFS becomes writable.
	 * Unfortunately, there's no asynchronous signal to detect readiness,
	 * but a 100ms delay has proven to be a stable workaround.
	 */
	k_sleep(K_MSEC(100));
#endif

	// delete old file if it exists
	fs_unlink("/SD:/data.pb");

	// open file
	ret = fs_open(&ctx->file, "/SD:/data.pb", FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
	if (ret < 0) {
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("writer init");
	return ret;
};

static int log_sdcard_writer_fini(struct context *ctx)
{
	int ret = 0;

	ret = fs_close(&ctx->file);
	if (ret != 0) {
		LOG_ERR("failed to close file");
	}

	ret = fs_unmount(&mp);
	if (ret < 0) {
		LOG_ERR("failed to unmount disk");
	}

	k_sem_give(&ctx->running);
	LOG_INF("writer fini");
	return ret;
};

static void log_sdcard_writer_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = 0;

	// constructor
	ret = log_sdcard_writer_init(ctx);
	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	// while running
	size_t size;
	size_t size_written;
	int64_t last_ticks = 0;
	uint8_t *data;
	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		/* Wait until the ring buffer has at least BUF_SIZE bytes available.
		 * We aim to write in chunks of BUF_SIZE, ideally matching the filesystem's
		 * sector size to maximize write throughput.
		 */
		if (ring_buf_size_get(&rb_sdcard) < BUF_SIZE) {
			k_msleep(1);
			continue;
		}

		size = ring_buf_get_claim(&rb_sdcard, &data, BUF_SIZE);
		if (size > 0) {
			LOG_DBG("writing: %d bytes to sdcard", size);
			size_written = fs_write(&ctx->file, data, size);
			ctx->total_size_written += size_written;
			ring_buf_get_finish(&rb_sdcard, size);
			if (size != size_written) {
				LOG_ERR_RATELIMIT_RATE(1000, "file write failed %d/%d",
						       size_written, size);
			}
		}
		int64_t now_ticks = k_uptime_ticks();
		if (now_ticks - last_ticks > 4 * CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
			LOG_DBG("fsync");
			last_ticks = now_ticks;
			fs_sync(&ctx->file);
		}
	}

	// deconstructor
	log_sdcard_writer_fini(ctx);
};

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				log_sdcard_writer_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "log_sdcard_writer");
	k_thread_start(tid);
	return 0;
}

static int log_sdcard_writer_cmd_handler(const struct shell *sh, size_t argc, char **argv,
					 void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			k_sem_give(&g_ctx.running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d size written: %10.3f MB",
			    (int)k_sem_count_get(&g_ctx.running) == 0,
			    ((double)ctx->total_size_written) / 1048576L);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_log_sdcard_writer, log_sdcard_writer_cmd_handler,
			     (start, &g_ctx, "start"), (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(log_sdcard_writer, &sub_log_sdcard_writer, "log_sdcard_writer commands", NULL);

static int log_sdcard_writer_sys_init(void)
{
	return start(&g_ctx);
	return 0;
};

SYS_INIT(log_sdcard_writer_sys_init, APPLICATION, 99);

// vi: ts=4 sw=4 et
