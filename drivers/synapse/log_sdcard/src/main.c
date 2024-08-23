/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>

#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <cerebri/core/perf_counter.h>

#include <pb_encode.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   1
#define BUF_SIZE      131072
#define TOPIC_RATE_HZ 10000

RING_BUF_DECLARE(rb_sdcard, BUF_SIZE);

LOG_MODULE_REGISTER(log_sdcard, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	// zros node handle
	struct zros_node node;
	// subscriptions
	struct zros_sub sub_imu;
	struct zros_sub sub_imu_q31_array;
	struct zros_sub sub_pwm;
	struct zros_sub sub_input;
	// file
	synapse_pb_Frame frame;
	// status
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	struct perf_counter perf;
};

static struct context g_ctx = {
	.node = {},
	.sub_imu = {},
	.sub_imu_q31_array = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.perf = {},
};

static int log_sdcard_init(struct context *ctx)
{
	int ret = 0;
	// initialize node
	zros_node_init(&ctx->node, "log_sdcard");
	perf_counter_init(&ctx->perf, "log imu", 1.0 / 100);

	// initialize node subscriptions
	ret = zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->frame.msg, TOPIC_RATE_HZ);
	if (ret < 0) {
		LOG_ERR("init imu failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_imu_q31_array, &ctx->node, &topic_imu_q31_array,
			    &ctx->frame.msg, TOPIC_RATE_HZ);
	if (ret < 0) {
		LOG_ERR("init imu_q31_array failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_pwm, &ctx->node, &topic_pwm, &ctx->frame.msg, TOPIC_RATE_HZ);
	if (ret < 0) {
		LOG_ERR("init pwm failed: %d", ret);
		return ret;
	}

	ret = zros_sub_init(&ctx->sub_input, &ctx->node, &topic_input, &ctx->frame.msg,
			    TOPIC_RATE_HZ);
	if (ret < 0) {
		LOG_ERR("init input failed: %d", ret);
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);

	// make sure writer is ready
	k_msleep(1000);
	LOG_INF("init");
	return ret;
};

static int log_sdcard_fini(struct context *ctx)
{
	int ret = 0;

	// close subscriptions
	zros_sub_fini(&ctx->sub_pwm);
	zros_sub_fini(&ctx->sub_input);
	zros_sub_fini(&ctx->sub_imu);
	zros_sub_fini(&ctx->sub_imu_q31_array);
	zros_node_fini(&ctx->node);

	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return ret;
};

static void log_sdcard_write_frame(struct context *ctx)
{
	static uint8_t buf[8192];
	size_t size_written, size_available;
	pb_ostream_t stream = pb_ostream_from_buffer(buf, ARRAY_SIZE(buf));
	if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, &ctx->frame, PB_ENCODE_DELIMITED)) {
		LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
	} else {
		size_available = ring_buf_space_get(&rb_sdcard);
		if (size_available < stream.bytes_written) {
			LOG_WRN("dropping packet, stream full");
		} else {
			size_written = ring_buf_put(&rb_sdcard, buf, stream.bytes_written);
			if (size_written != stream.bytes_written) {
				LOG_INF("partial write: %d/%d", size_written, stream.bytes_written);
				return;
			}
			// LOG_INF("writing %d", stream.bytes_written);
		}
	}
}

static void log_sdcard_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = 0;

	// constructor
	ret = log_sdcard_init(ctx);
	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	// subscribe to topics

	// while running
	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		struct k_poll_event events[] = {
			*zros_sub_get_event(&ctx->sub_imu),
			*zros_sub_get_event(&ctx->sub_imu_q31_array),
		};

		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(2000));
		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		perf_counter_update(&ctx->perf);

		if (zros_sub_update_available(&ctx->sub_imu)) {
			zros_sub_update(&ctx->sub_imu);
			ctx->frame.which_msg = synapse_pb_Frame_imu_tag;
			snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), "imu");
			log_sdcard_write_frame(ctx);
		}

		if (zros_sub_update_available(&ctx->sub_imu_q31_array)) {
			zros_sub_update(&ctx->sub_imu_q31_array);
			ctx->frame.which_msg = synapse_pb_Frame_imu_q31_array_tag;
			snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), "imu_q31_array");
			log_sdcard_write_frame(ctx);
		}

		if (zros_sub_update_available(&ctx->sub_input)) {
			zros_sub_update(&ctx->sub_input);
			ctx->frame.which_msg = synapse_pb_Frame_input_tag;
			snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), "input");
			log_sdcard_write_frame(ctx);
		}

		if (zros_sub_update_available(&ctx->sub_pwm)) {
			zros_sub_update(&ctx->sub_pwm);
			ctx->frame.which_msg = synapse_pb_Frame_pwm_tag;
			snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), "pwm");
			log_sdcard_write_frame(ctx);
		}
	}

	// deconstructor
	log_sdcard_fini(ctx);
};

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      log_sdcard_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "log_sdcard");
	k_thread_start(tid);
	return 0;
}

static int log_sdcard_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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
		shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_log_sdcard, log_sdcard_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(log_sdcard, &sub_log_sdcard, "log_sdcard commands", NULL);

static int log_sdcard_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(log_sdcard_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
