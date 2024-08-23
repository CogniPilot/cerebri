/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <synapse_topic_list.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/casadi.h>

#include "app/rdd2/casadi/rdd2.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY   4

LOG_MODULE_REGISTER(rdd2_angular_velocity, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	synapse_pb_Status status;
	synapse_pb_Vector3 angular_velocity_sp, moment_sp, moment_ff;
	synapse_pb_Odometry odometry_estimator;
	struct zros_sub sub_status, sub_angular_velocity_sp, sub_odometry_estimator, sub_moment_ff;
	struct zros_pub pub_moment_sp;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	double dt;
	double kp[3];
	double ki[3];
	double kd[3];
	double i_max[3];
	double f_cut;
	double omega[3];
	double omega_r[3];
	double omega_i[3];
	double omega_e[3];
	double domega_e[3];
	double alpha;
};

static struct context g_ctx = {
	.node = {},
	.status = synapse_pb_Status_init_default,
	.moment_sp = synapse_pb_Vector3_init_default,
	.angular_velocity_sp = synapse_pb_Vector3_init_default,
	.moment_ff = synapse_pb_Vector3_init_default,
	.sub_status = {},
	.sub_angular_velocity_sp = {},
	.sub_odometry_estimator = {},
	.sub_moment_ff = {},
	.pub_moment_sp = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.kp =
		{
			CONFIG_CEREBRI_RDD2_ROLLRATE_KP * 1e-6,
			CONFIG_CEREBRI_RDD2_PITCHRATE_KP * 1e-6,
			CONFIG_CEREBRI_RDD2_YAWRATE_KP * 1e-6,
		},
	.ki =
		{
			CONFIG_CEREBRI_RDD2_ROLLRATE_KI * 1e-6,
			CONFIG_CEREBRI_RDD2_PITCHRATE_KI * 1e-6,
			CONFIG_CEREBRI_RDD2_YAWRATE_KI * 1e-6,
		},
	.kd =
		{
			CONFIG_CEREBRI_RDD2_ROLLRATE_KD * 1e-6,
			CONFIG_CEREBRI_RDD2_PITCHRATE_KD * 1e-6,
			CONFIG_CEREBRI_RDD2_YAWRATE_KD * 1e-6,
		},
	.i_max =
		{
			CONFIG_CEREBRI_RDD2_ROLLRATE_IMAX * 1e-6,
			CONFIG_CEREBRI_RDD2_PITCHRATE_IMAX * 1e-6,
			CONFIG_CEREBRI_RDD2_YAWRATE_IMAX * 1e-6,
		},
	.f_cut = CONFIG_CEREBRI_RDD2_ATTITUDE_RATE_FCUT * 1e-3,
	.omega_i = {},
	.omega_e = {},
	.domega_e = {},
};

static void rdd2_angular_velocity_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_angular_velocity");
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_angular_velocity_sp, &ctx->node, &topic_angular_velocity_sp,
		      &ctx->angular_velocity_sp, 1000);
	zros_sub_init(&ctx->sub_odometry_estimator, &ctx->node, &topic_odometry_estimator,
		      &ctx->odometry_estimator, 1000);
	zros_sub_init(&ctx->sub_moment_ff, &ctx->node, &topic_moment_ff, &ctx->moment_ff, 1000);
	zros_pub_init(&ctx->pub_moment_sp, &ctx->node, &topic_moment_sp, &ctx->moment_sp);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_angular_velocity_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_angular_velocity_sp);
	zros_sub_fini(&ctx->sub_odometry_estimator);
	zros_sub_fini(&ctx->sub_moment_ff);
	zros_pub_fini(&ctx->pub_moment_sp);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void rdd2_angular_velocity_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	rdd2_angular_velocity_init(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_odometry_estimator),
	};

	int64_t ticks_last = k_uptime_ticks();

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		// wait for estimator odometry, publish at 10 Hz regardless
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(100));
		if (rc != 0) {
			LOG_DBG("not receiving estimator odometry");
		}

		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}

		if (zros_sub_update_available(&ctx->sub_odometry_estimator)) {
			zros_sub_update(&ctx->sub_odometry_estimator);
		}

		if (zros_sub_update_available(&ctx->sub_angular_velocity_sp)) {
			zros_sub_update(&ctx->sub_angular_velocity_sp);
		}

		if (zros_sub_update_available(&ctx->sub_moment_ff)) {
			zros_sub_update(&ctx->sub_moment_ff);
		}

		// calculate dt
		int64_t ticks_now = k_uptime_ticks();
		ctx->dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		ticks_last = ticks_now;
		if (ctx->dt < 0 || ctx->dt > 0.1) {
			LOG_DBG("odometry rate too low");
			continue;
		}

		double M[3];
		{
			// attitude_rate_control:(
			//  kp[3],ki[3],kd[3],f_cut,i_max[3],
			//  omega[3],omega_r[3],i0[3],e0[3],de0[3],dt)->(M[3],i1[3],e1[3],de1[3]) */
			CASADI_FUNC_ARGS(attitude_rate_control);
			ctx->omega[0] = ctx->odometry_estimator.twist.angular.x;
			ctx->omega[1] = ctx->odometry_estimator.twist.angular.y;
			ctx->omega[2] = ctx->odometry_estimator.twist.angular.z;
			ctx->omega_r[0] = ctx->angular_velocity_sp.x;
			ctx->omega_r[1] = ctx->angular_velocity_sp.y;
			ctx->omega_r[2] = ctx->angular_velocity_sp.z;
			args[0] = ctx->kp;
			args[1] = ctx->ki;
			args[2] = ctx->kd;
			args[3] = &(ctx->f_cut);
			args[4] = ctx->i_max;
			args[5] = ctx->omega;
			args[6] = ctx->omega_r;
			args[7] = ctx->omega_i;
			args[8] = ctx->omega_e;
			args[9] = ctx->domega_e;
			args[10] = &ctx->dt;
			res[0] = M;
			res[1] = ctx->omega_i;
			res[2] = ctx->omega_e;
			res[3] = ctx->domega_e;
			res[4] = &ctx->alpha;
			CASADI_FUNC_CALL(attitude_rate_control);
		}

		// LOG_INF("omega_i: %10.4f %10.4f %10.4f",
		//     omega_i[0], omega_i[1], omega_i[2]);

		bool data_ok = true;
		for (int i = 0; i < 3; i++) {
			if (!isfinite(ctx->omega_i[i])) {
				LOG_ERR("omega_i[%d] not finite: %10.4f", i, ctx->omega_i[i]);
				data_ok = false;
				break;
			}
			if (!isfinite(M[i])) {
				LOG_ERR("M[%d] not finite: %10.4f", i, M[i]);
				data_ok = false;
				break;
			}
		}

		// publish moment setpoint
		if (data_ok) {
			ctx->moment_sp.x = M[0] + ctx->moment_ff.x;
			ctx->moment_sp.y = M[1] + ctx->moment_ff.y;
			ctx->moment_sp.z = M[2] + ctx->moment_ff.z;
			zros_pub_update(&ctx->pub_moment_sp);
		}
	}

	rdd2_angular_velocity_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      rdd2_angular_velocity_run, ctx, NULL, NULL, MY_PRIORITY, 0,
				      K_FOREVER);
	k_thread_name_set(tid, "rdd2_angular_velocity");
	k_thread_start(tid);
	return 0;
}

static int rdd2_angular_velocity_cmd_handler(const struct shell *sh, size_t argc, char **argv,
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
		shell_print(sh, "running\t: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
		shell_print(sh, "dt\t: %10.6f", ctx->dt);
		shell_print(sh, "kp\t: %10.6f %10.6f %10.6f", ctx->kp[0], ctx->kp[1], ctx->kp[2]);
		shell_print(sh, "ki\t: %10.6f %10.6f %10.6f", ctx->ki[0], ctx->ki[1], ctx->ki[2]);
		shell_print(sh, "kd\t: %10.6f %10.6f %10.6f", ctx->kd[0], ctx->kd[1], ctx->kd[2]);
		shell_print(sh, "omega\t: %10.6f %10.6f %10.6f", ctx->omega[0], ctx->omega[1],
			    ctx->omega[2]);
		shell_print(sh, "omega_r\t: %10.6f %10.6f %10.6f", ctx->omega_r[0], ctx->omega_r[1],
			    ctx->omega_r[2]);
		shell_print(sh, "omega_i\t: %10.6f %10.6f %10.6f", ctx->omega_i[0], ctx->omega_i[1],
			    ctx->omega_i[2]);
		shell_print(sh, "omega_e\t: %10.6f %10.6f %10.6f", ctx->omega_e[0], ctx->omega_e[1],
			    ctx->omega_e[2]);
		shell_print(sh, "domega_e: %10.6f %10.6f %10.6f", ctx->domega_e[0],
			    ctx->domega_e[1], ctx->domega_e[2]);
		shell_print(sh, "i_max\t: %10.6f %10.6f %10.6f", ctx->i_max[0], ctx->i_max[1],
			    ctx->i_max[2]);
		shell_print(sh, "f_cut\t: %10.6f", ctx->f_cut);
		shell_print(sh, "alpha\t: %10.6f", ctx->alpha);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_angular_velocity, rdd2_angular_velocity_cmd_handler,
			     (start, &g_ctx, "start"), (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_angular_velocity, &sub_rdd2_angular_velocity,
		   "rdd2 angular velocity commands", NULL);

static int rdd2_angular_velocity_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_angular_velocity_sys_init, APPLICATION, 4);

// vi: ts=4 sw=4 et
