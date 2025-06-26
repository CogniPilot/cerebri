/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include "proto/udp_rx.h"
#include <synapse_topic_list.h>

#include <pb_decode.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY   1

LOG_MODULE_REGISTER(eth_rx, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	synapse_pb_Frame rx_frame;
	struct udp_rx udp;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.rx_frame = synapse_pb_Frame_init_default,
	.udp = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void handle_frame(struct context *ctx)
{
	synapse_pb_Frame *frame = &ctx->rx_frame;
	void *msg = NULL;
	struct zros_topic *topic = NULL;

	// determine topic for data
	if (frame->which_msg == synapse_pb_Frame_bezier_trajectory_tag) {
		msg = &frame->msg.bezier_trajectory;
		topic = &topic_bezier_trajectory_ethernet;
	} else if (frame->which_msg == synapse_pb_Frame_clock_offset_tag) {
		msg = &frame->msg.clock_offset;
		topic = &topic_clock_offset_ethernet;
	} else if (frame->which_msg == synapse_pb_Frame_input_tag) {
		msg = &frame->msg.input;
		topic = &topic_input_ethernet;
	} else if (frame->which_msg == synapse_pb_Frame_twist_tag) {
		msg = &frame->msg.twist;
		topic = &topic_cmd_vel_ethernet;
	} else if (frame->which_msg == synapse_pb_Frame_odometry_tag) {
		msg = &frame->msg.odometry;
		topic = &topic_odometry_ethernet;
#ifdef CONFIG_CEREBRI_DREAM_HIL
	} else if (frame->which_msg == synapse_pb_Frame_battery_state_tag) {
		msg = &frame->msg.battery_state;
		topic = &topic_battery_state;
	} else if (frame->which_msg == synapse_pb_Frame_imutag) {
		msg = &frame->msg.imu;
		topic = &topic_imu;
	} else if (frame->which_msg == synapse_pb_Frame_magnetic_fieldtag) {
		msg = &frame->msg.magnetic_field;
		topic = &topic_magnetic_field;
	} else if (frame->which_msg == synapse_pb_Frame_nav_sat_fixtag) {
		msg = &frame->msg.nav_sat_fix;
		zros_topic_publish(&topic_nav_sat_fix, msg);
	} else if (frame->which_msg == synapse_pb_Frame_wheel_odometrytag) {
		msg = &frame->msg.wheel_odometry;
		topic = &topic_wheel_odometry;
#endif
	}

	if (msg == NULL) {
		LOG_ERR("unhandled message: %d", frame->which_msg);
	} else {
		int ret = zros_topic_publish(topic, msg);
		if (ret != 0) {
			LOG_ERR("failed to publish msg: %d", frame->which_msg);
		}
	}
}

static int eth_rx_init(struct context *ctx)
{
	int ret = 0;
	// setup zros node
	zros_node_init(&ctx->node, "eth_rx");

	// setup udp connection
	ret = udp_rx_init(&ctx->udp);
	if (ret < 0) {
		LOG_ERR("udp rx init failed: %d", ret);
		return ret;
	}

	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("started");
	return ret;
};

static int eth_rx_fini(struct context *ctx)
{
	int ret = 0;
	// close udp socket
	ret = udp_rx_fini(&ctx->udp);

	// close subscriptions
	zros_node_fini(&ctx->node);

	k_sem_give(&ctx->running);
	LOG_INF("stopped");
	return ret;
};

static void eth_rx_run(void *p0, void *p1, void *p2)
{
	int ret = 0;
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	// constructor
	ret = eth_rx_init(ctx);
	if (ret < 0) {
		LOG_ERR("init failed: %d", ret);
		return;
	}

	LOG_INF("running");
	pb_istream_t stream;

	// while running
	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		// poll sockets and receive data
		int received = udp_rx_receive(&ctx->udp);
		if (received < 0) {
			LOG_ERR("connection error: %d", errno);
		} else if (received > 0) {
			stream = pb_istream_from_buffer(ctx->udp.rx_buf, received);
			while (stream.bytes_left > 0) {
				if (!pb_decode_ex(&stream, synapse_pb_Frame_fields, &ctx->rx_frame,
						  PB_DECODE_DELIMITED)) {
					LOG_ERR("failed to decode msg: %s\n",
						PB_GET_ERROR(&stream));
				} else {
					handle_frame(ctx);
				}
			}
		}
	}

	// deconstructor
	ret = eth_rx_fini(ctx);
	if (ret < 0) {
		LOG_ERR("fini failed: %d", ret);
	}
};

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      eth_rx_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "eth_rx");
	k_thread_start(tid);
	return 0;
}

static int eth_rx_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_eth_rx, eth_rx_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(eth_rx, &sub_eth_rx, "eth_rx commands", NULL);

static int eth_rx_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(eth_rx_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
