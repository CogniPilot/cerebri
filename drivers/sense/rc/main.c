/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   2

LOG_MODULE_REGISTER(sense_rc, CONFIG_CEREBRI_SENSE_RC_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	struct zros_pub pub_input;
	synapse_pb_Input input;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	int last_event;
};

static struct context g_ctx = {
	.node = {},
	.pub_input = {},
	.input = {.has_timestamp = true, .channel_count = 16, .channel = {}},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.last_event = 0,
};

static void sense_rc_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "sense_rc");
	zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input_rc, &ctx->input);
	ctx->last_event = 0;
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void sense_rc_fini(struct context *ctx)
{
	zros_pub_fini(&ctx->pub_input);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void sense_rc_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	sense_rc_init(ctx);

	// wait for stop request
	while (k_sem_take(&ctx->running, K_MSEC(1000)) < 0)
		;

	sense_rc_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      sense_rc_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "sense_rc");
	k_thread_start(tid);
	return 0;
}

static void input_cb(struct input_event *evt, void *userdata)
{
	struct context *ctx = userdata;

	// check if still running
	if (k_sem_count_get(&ctx->running) != 0) {
		return;
	}

	float x0 = 1024;
	float scale = 784;

	if (evt->code > 0 && evt->code <= ctx->input.channel_count) {
		ctx->input.channel[evt->code - 1] = (evt->value - x0) / scale;
	} else {
		LOG_DBG("unhandled event: %d %d %d %d", evt->code, evt->sync, evt->type,
			evt->value);
	}

	if (evt->sync == true) {
		stamp_msg(&ctx->input.timestamp, k_uptime_ticks());
		zros_pub_update(&ctx->pub_input);
	}
	ctx->last_event = evt->code;
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(rc)), input_cb, &g_ctx);

static int sense_rc_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_rc, sense_rc_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_rc, &sub_sense_rc, "sense rc args", NULL);

static int sense_rc_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(sense_rc_sys_init, APPLICATION, 2);

/* vi: ts=4 sw=4 et */
