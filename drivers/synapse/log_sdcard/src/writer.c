/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <ff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

#define FS_RET_OK     FR_OK
#define MY_STACK_SIZE 8192
#define MY_PRIORITY   1
#define BUF_SIZE      (131072 * 2)

extern struct ring_buf rb_sdcard;

static FATFS fat_fs;
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
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
	/* raw disk i/o */
	static const char *disk_pdrv = "SD";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	if (disk_access_init(disk_pdrv) != 0) {
		LOG_ERR("Storage init ERROR!");
		return -1;
	}

	if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
		LOG_ERR("Unable to get sector count");
		return -1;
	}
	LOG_INF("Block count %u", block_count);

	if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
		LOG_ERR("Unable to get sector size");
		return -1;
	}
	LOG_INF("Sector size %u", block_size);

	memory_size_mb = (uint64_t)block_count * block_size;
	LOG_INF("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		LOG_INF("Disk mounted.");
	} else {
		LOG_INF("Failed to mount disk - trying one more time");
		res = fs_mount(&mp);
		if (res != FR_OK) {
			LOG_INF("Error mounting disk.");
			return -1;
		}
	}
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

	// delete old file if it exists
	fs_unlink("/SD:/data.pb");

	// open file
	ret = fs_open(&ctx->file, "/SD:/data.pb", FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
	if (ret < 0) {
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
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
	LOG_INF("fini");
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

		k_msleep(1);
		size = ring_buf_get_claim(&rb_sdcard, &data, BUF_SIZE);
		if (size > 0) {
			// LOG_INF("writing: %d bytes to sdcard", size);
			size_written = fs_write(&ctx->file, data, size);
			ctx->total_size_written += size_written;
			ring_buf_get_finish(&rb_sdcard, size);
			if (size != size_written) {
				LOG_ERR("file write failed %d/%d", size_written, size);
			}
		}
		int64_t now_ticks = k_uptime_ticks();
		if (now_ticks - last_ticks > 4 * CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
			// LOG_INF("fsync");
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

SYS_INIT(log_sdcard_writer_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
