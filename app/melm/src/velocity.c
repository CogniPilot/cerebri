/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app/melm/casadi/melm.h"
#include "math.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/casadi.h>

#include "mixing.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY   4

LOG_MODULE_REGISTER(melm_velocity, CONFIG_CEREBRI_MELM_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	synapse_pb_Twist cmd_vel;
	synapse_pb_Status status;
	synapse_pb_Actuators actuators;
	struct zros_sub sub_status, sub_cmd_vel;
	struct zros_pub pub_actuators;
	const double wheel_radius;
	const double wheel_base;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.cmd_vel = synapse_pb_Twist_init_default,
	.status = synapse_pb_Status_init_default,
	.actuators = synapse_pb_Actuators_init_default,
	.sub_status = {},
	.sub_cmd_vel = {},
	.pub_actuators = {},
	.wheel_radius = CONFIG_CEREBRI_MELM_WHEEL_RADIUS_MM / 1000.0,
	.wheel_base = CONFIG_CEREBRI_MELM_WHEEL_BASE_MM / 1000.0,
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void melm_velocity_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "melm_velocity");
	zros_sub_init(&ctx->sub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel, 10);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void melm_velocity_fini(struct context *ctx)
{
	zros_pub_fini(&ctx->pub_actuators);
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_cmd_vel);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

// computes actuators from cmd_vel
static void melm_velocity_update(struct context *ctx)
{
	double V = ctx->cmd_vel.linear.x;
	double omega = ctx->cmd_vel.angular.z;
	double width = 0.5; // width
	double Vw = 0; // differential wheel velocity

	CASADI_FUNC_ARGS(differential_steering);
	args[0] = &ctx->wheel_base;
	args[1] = &omega;
	args[2] = &width;
	res[0] = &Vw;
	CASADI_FUNC_CALL(differential_steering);

	double omega_left = (V - Vw) / ctx->wheel_radius;;
	double omega_right = (V + Vw) / ctx->wheel_radius;;

	bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;

	melm_set_actuators(&ctx->actuators, omega_left, omega_right, armed);

	// publish
	zros_pub_update(&ctx->pub_actuators);
}

static void melm_velocity_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	melm_velocity_init(ctx);

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		struct k_poll_event events[] = {
			*zros_sub_get_event(&ctx->sub_cmd_vel),
		};
		int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

		if (rc < 0) {
			LOG_DBG("not receiving cmd_vel");
			continue;
		}

		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}

		if (zros_sub_update_available(&ctx->sub_cmd_vel)) {
			zros_sub_update(&ctx->sub_cmd_vel);
		}

		// handle modes
		if (ctx->status.mode != synapse_pb_Status_Mode_MODE_ACTUATORS) {
			melm_velocity_update(ctx);
		}
	}

	melm_velocity_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				melm_velocity_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "melm_velocity");
	k_thread_start(tid);
	return 0;
}

static int melm_velocity_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_melm_velocity, melm_velocity_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(melm_velocity, &sub_melm_velocity, "melm velocity arguments", NULL);

static int melm_velocity_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(melm_velocity_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
