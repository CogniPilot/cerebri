/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app/rdd2/casadi/rdd2.h"
#include "math.h"

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>
#include <cerebri/core/log_utils.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY   4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CEREBRI_NODE_LOG_INIT(rdd2_allocation, LOG_LEVEL_WRN);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	synapse_pb_Status status;
	synapse_pb_Actuators actuators;
	synapse_pb_Vector3 force_sp, moment_sp;
	struct zros_sub sub_status, sub_force_sp, sub_moment_sp;
	struct zros_pub pub_actuators;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.status = synapse_pb_Status_init_default,
	.actuators =
		{
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.velocity_count = 4,
			.normalized_count = 0,
			.position_count = 0,
			.position = {},
			.normalized = {},
			.velocity = {},
		},
	.sub_status = {},
	.sub_force_sp = {},
	.sub_moment_sp = {},
	.pub_actuators = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void rdd2_allocation_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_allocation");
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp, 1000);
	zros_sub_init(&ctx->sub_moment_sp, &ctx->node, &topic_moment_sp, &ctx->moment_sp, 1000);
	zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_allocation_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_force_sp);
	zros_sub_fini(&ctx->sub_moment_sp);
	zros_pub_fini(&ctx->pub_actuators);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void stop(struct context *ctx)
{
	for (int i = 0; i < 4; i++) {
		ctx->actuators.velocity[i] = 0;
	}
}

static void rdd2_allocation_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	rdd2_allocation_init(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_moment_sp),
	};

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(100));
		if (rc != 0) {
			LOG_DBG("not receiving moment_sp");
		}

		// update subscriptions
		zros_sub_update(&ctx->sub_status);
		zros_sub_update(&ctx->sub_force_sp);
		zros_sub_update(&ctx->sub_moment_sp);

		if (rc < 0) {
			stop(ctx);
			LOG_DBG("no data, stopped");
		} else if (ctx->status.arming != synapse_pb_Status_Arming_ARMING_ARMED) {
			// not armed, stop
			stop(ctx);
		} else {
			static double const F_max = 20.0;
			static double const l = CONFIG_CEREBRI_RDD2_MOTOR_L_MM * 1e-3;
			static double const Cm = CONFIG_CEREBRI_RDD2_MOTOR_CM * 1e-6;
			static double const Ct = CONFIG_CEREBRI_RDD2_MOTOR_CT * 1e-9;
			double omega[4];
			double Fp_sum[4], F_moment[4], F_thrust[4], M_sat[3];
			double moment[3] = {ctx->moment_sp.x, ctx->moment_sp.y, ctx->moment_sp.z};

			// control_allocation:(F_max,l,Cm,Ct,T,M[3])->(omega[4],Fp_sum[4],F_moment[4],F_thrust[4],M_sat[3])
			CASADI_FUNC_ARGS(control_allocation)

			args[0] = &F_max;
			args[1] = &l;
			args[2] = &Cm;
			args[3] = &Ct;
			args[4] = &ctx->force_sp.z;
			args[5] = moment;

			res[0] = omega;
			res[1] = Fp_sum;
			res[2] = F_moment;
			res[3] = F_thrust;
			res[4] = M_sat;
			CASADI_FUNC_CALL(control_allocation)

			for (int i = 0; i < 4; i++) {
				if (!isfinite(omega[i])) {
					LOG_WRN("omega is not finite: %10.4f", omega[i]);
					omega[i] = 0;
				} else if (omega[i] > 3000) {
					LOG_WRN("omega too large: %10.4f", omega[i]);
					omega[i] = 3000;
				} else if (omega[i] < 0) {
					LOG_WRN("omega negative: %10.4f", omega[i]);
					omega[i] = 0;
				}
				ctx->actuators.velocity[i] = omega[i];
			}
		}

		stamp_msg(&ctx->actuators.stamp, k_uptime_ticks());

		// publish
		zros_pub_update(&ctx->pub_actuators);
	}

	rdd2_allocation_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				rdd2_allocation_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "rdd2_allocation");
	k_thread_start(tid);
	return 0;
}

static int rdd2_allocation_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_allocation, rdd2_allocation_cmd_handler,
			     (start, &g_ctx, "start"), (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_allocation, &sub_rdd2_allocation, "rdd2 allocation commands", NULL);

static int rdd2_allocation_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_allocation_sys_init, APPLICATION, 4);

// vi: ts=4 sw=4 et
