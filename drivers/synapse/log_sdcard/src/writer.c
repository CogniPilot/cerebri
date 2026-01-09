/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <time.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <log_sdcard.h>

#if CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#endif

#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#ifdef CONFIG_FILE_SYSTEM_RPMSGFS
#include <synapse_rpmsg.h>
#endif

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>
#include <synapse_topic_list.h>

#define FS_RET_OK     FR_OK
#define MY_STACK_SIZE 2048
#define MY_PRIORITY   1
#define BUF_SIZE      512

/* Minimum valid epoch timestamp (2020-01-01 00:00:00 UTC) */
#define MIN_VALID_EPOCH 1577836800

/* Maximum filename length: /SD:/YYYY-MM-DD_HHMMSS.pb = 27 chars + null
 * Using 64 to satisfy compiler warning about potential truncation with large int values
 */
#define MAX_FILENAME_LEN 64

extern struct ring_buf rb_sdcard;

#ifndef CONFIG_FILE_SYSTEM_RPMSGFS
static FATFS fat_fs;
#endif
static struct fs_mount_t mp = {
	.type = FS_FATFS,
};

LOG_MODULE_DECLARE(log_sdcard, CONFIG_CEREBRI_SYNAPSE_LOG_SDCARD_LOG_LEVEL);

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

/* ZROS node and subscriber for NavSatFix */
static struct zros_node g_node;
static struct zros_sub g_sub_nav_sat_fix;
static synapse_pb_NavSatFix g_nav_sat_fix;
static bool g_zros_initialized = false;

/**
 * @brief Convert epoch seconds to a formatted filename
 * @param epoch_secs Unix epoch seconds
 * @param buf Output buffer for filename
 * @param len Buffer length
 */
static void epoch_to_filename(int64_t epoch_secs, char *buf, size_t len)
{
	time_t t = (time_t)epoch_secs;
	struct tm *tm = gmtime(&t);
	if (tm == NULL) {
		snprintf(buf, len, "/SD:/data.pb");
		return;
	}
	snprintf(buf, len, "/SD:/%04d-%02d-%02d_%02d%02d%02d.pb", tm->tm_year + 1900,
		 tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

/**
 * @brief Find the next available sequence number for log files
 * @return Next available sequence number (1-9999)
 */
static int find_next_sequence_number(void)
{
	struct fs_dir_t dir;
	struct fs_dirent entry;
	int max_seq = 0;

	fs_dir_t_init(&dir);
	if (fs_opendir(&dir, disk_mount_pt) != 0) {
		return 1;
	}

	while (fs_readdir(&dir, &entry) == 0 && entry.name[0] != '\0') {
		int seq;
		/* Match pattern log_NNNN.pb */
		if (sscanf(entry.name, "log_%d.pb", &seq) == 1) {
			if (seq > max_seq) {
				max_seq = seq;
			}
		}
	}

	fs_closedir(&dir);
	return max_seq + 1;
}

/**
 * @brief Generate a log filename based on GPS time or sequence number
 * @param buf Output buffer for filename
 * @param len Buffer length
 */
static void generate_log_filename(char *buf, size_t len)
{
	/* Initialize ZROS subscription if not done */
	if (!g_zros_initialized) {
		zros_node_init(&g_node, "log_sdcard_writer");
		zros_sub_init(&g_sub_nav_sat_fix, &g_node, &topic_nav_sat_fix, &g_nav_sat_fix, 1);
		g_zros_initialized = true;
	}

	/* Try to get GPS time from NavSatFix topic */
	if (zros_sub_update_available(&g_sub_nav_sat_fix)) {
		zros_sub_update(&g_sub_nav_sat_fix);
	}

	/* Check if we have a valid GPS timestamp */
	if (g_nav_sat_fix.has_stamp && g_nav_sat_fix.stamp.seconds > MIN_VALID_EPOCH) {
		epoch_to_filename(g_nav_sat_fix.stamp.seconds, buf, len);
		LOG_INF("Using GPS timestamp for log file: %s", buf);
	} else {
		/* Fallback to sequential naming */
		int seq = find_next_sequence_number();
		snprintf(buf, len, "/SD:/log_%04d.pb", seq);
		LOG_INF("No GPS fix, using sequential log file: %s", buf);
	}
}

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
	char filename[MAX_FILENAME_LEN];

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

	/* Generate filename based on GPS time or sequence number */
	generate_log_filename(filename, sizeof(filename));

	/* Open log file (never overwrites existing files) */
	ret = fs_open(&ctx->file, filename, FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
	if (ret < 0) {
		LOG_ERR("Failed to open log file: %s (%d)", filename, ret);
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("writer init: %s", filename);
	return ret;
}

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

/* Public API functions */
int log_sdcard_writer_start(void)
{
	if (k_sem_count_get(&g_ctx.running) == 0) {
		return -EALREADY;
	}
	return start(&g_ctx);
}

int log_sdcard_writer_stop(void)
{
	if (k_sem_count_get(&g_ctx.running) != 0) {
		return -EALREADY;
	}
	k_sem_give(&g_ctx.running);
	return 0;
}

bool log_sdcard_writer_is_running(void)
{
	return k_sem_count_get(&g_ctx.running) == 0;
}

static int log_sdcard_writer_cmd_handler(const struct shell *sh, size_t argc, char **argv,
					 void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		int ret = log_sdcard_writer_start();
		if (ret == -EALREADY) {
			shell_print(sh, "already running");
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		int ret = log_sdcard_writer_stop();
		if (ret == -EALREADY) {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d size written: %10.3f MB",
			    log_sdcard_writer_is_running(),
			    ((double)ctx->total_size_written) / 1048576L);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_log_sdcard_writer, log_sdcard_writer_cmd_handler,
			     (start, &g_ctx, "start"), (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(log_sdcard_writer, &sub_log_sdcard_writer, "log_sdcard_writer commands", NULL);

/* Note: Auto-start removed - logging is now controlled by arming state */

// vi: ts=4 sw=4 et
