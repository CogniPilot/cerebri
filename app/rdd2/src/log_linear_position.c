/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "app/rdd2/casadi/rdd2.h"
#include "app/rdd2/casadi/rdd2_loglinear.h"

#include <cerebri/core/casadi.h>
#include <cerebri/core/log_utils.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY   4

CEREBRI_NODE_LOG_INIT(rdd2_position, LOG_LEVEL_WRN);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

static const double thrust_trim = CONFIG_CEREBRI_RDD2_THRUST_TRIM * 1e-3;

struct context {
	struct zros_node node;
	synapse_pb_Status status;
	synapse_pb_Odometry odometry_estimator;
	synapse_pb_Vector3 force_sp, position_sp, velocity_sp, accel_sp;
	synapse_pb_Quaternion attitude_sp, orientation_sp;
	struct zros_sub sub_status, sub_position_sp, sub_velocity_sp, sub_accel_sp,
		sub_odometry_estimator, sub_orientation_sp;
	struct zros_pub pub_force_sp, pub_attitude_sp;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.status = synapse_pb_Status_init_default,
	.force_sp = synapse_pb_Vector3_init_default,
	.odometry_estimator = synapse_pb_Odometry_init_default,
	.attitude_sp = synapse_pb_Quaternion_init_default,
	.orientation_sp = synapse_pb_Quaternion_init_default,
	.position_sp = synapse_pb_Vector3_init_default,
	.velocity_sp = synapse_pb_Vector3_init_default,
	.accel_sp = synapse_pb_Vector3_init_default,
	.sub_status = {},
	.sub_position_sp = {},
	.sub_velocity_sp = {},
	.sub_accel_sp = {},
	.sub_odometry_estimator = {},
	.sub_orientation_sp = {},
	.pub_force_sp = {},
	.pub_attitude_sp = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void rdd2_position_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_position");
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_position_sp, &ctx->node, &topic_position_sp, &ctx->position_sp, 10);
	zros_sub_init(&ctx->sub_velocity_sp, &ctx->node, &topic_velocity_sp, &ctx->velocity_sp, 10);
	zros_sub_init(&ctx->sub_accel_sp, &ctx->node, &topic_accel_sp, &ctx->accel_sp, 10);
	zros_sub_init(&ctx->sub_odometry_estimator, &ctx->node, &topic_odometry_estimator,
		      &ctx->odometry_estimator, 10);
	zros_sub_init(&ctx->sub_orientation_sp, &ctx->node, &topic_orientation_sp,
		      &ctx->orientation_sp, 10);
	zros_pub_init(&ctx->pub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp);
	zros_pub_init(&ctx->pub_attitude_sp, &ctx->node, &topic_attitude_sp, &ctx->attitude_sp);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_position_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_position_sp);
	zros_sub_fini(&ctx->sub_velocity_sp);
	zros_sub_fini(&ctx->sub_accel_sp);
	zros_sub_fini(&ctx->sub_odometry_estimator);
	zros_sub_fini(&ctx->sub_orientation_sp);
	zros_pub_fini(&ctx->pub_force_sp);
	zros_pub_fini(&ctx->pub_attitude_sp);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void rdd2_position_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	rdd2_position_init(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_odometry_estimator),
	};

	double dt = 0;
	int64_t ticks_last = k_uptime_ticks();
	double z_i[3]; // altitude error integral

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG("pos not receiving  pose");
			continue;
		}

		// update subscriptions
		zros_sub_update(&ctx->sub_status);
		zros_sub_update(&ctx->sub_position_sp);
		zros_sub_update(&ctx->sub_velocity_sp);
		zros_sub_update(&ctx->sub_accel_sp);
		zros_sub_update(&ctx->sub_orientation_sp);
		zros_sub_update(&ctx->sub_odometry_estimator);

		// calculate dt
		int64_t ticks_now = k_uptime_ticks();
		dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		ticks_last = ticks_now;
		if (dt < 0 || dt > 0.5) {
			LOG_WRN("position update rate too low");
			continue;
		}

		if (ctx->status.mode == synapse_pb_Status_Mode_MODE_POSITION ||
		    ctx->status.mode == synapse_pb_Status_Mode_MODE_VELOCITY ||
		    ctx->status.mode == synapse_pb_Status_Mode_MODE_ACCELERATION ||
		    ctx->status.mode == synapse_pb_Status_Mode_MODE_BEZIER) {

			// vehicle state
			double p_w[3] = {ctx->odometry_estimator.pose.position.x,
					 ctx->odometry_estimator.pose.position.y,
					 ctx->odometry_estimator.pose.position.z};

			double v_b[3] = {ctx->odometry_estimator.twist.linear.x,
					 ctx->odometry_estimator.twist.linear.y,
					 ctx->odometry_estimator.twist.linear.z};

			double q_wb[4] = {ctx->odometry_estimator.pose.orientation.w,
					  ctx->odometry_estimator.pose.orientation.x,
					  ctx->odometry_estimator.pose.orientation.y,
					  ctx->odometry_estimator.pose.orientation.z};

			// vehicle setpoint
			double pt_w[3] = {ctx->position_sp.x, ctx->position_sp.y,
					  ctx->position_sp.z};

			double vt_w[3] = {ctx->velocity_sp.x, ctx->velocity_sp.y,
					  ctx->velocity_sp.z};

			double at_w[3] = {ctx->accel_sp.x, ctx->accel_sp.y, ctx->accel_sp.z};

			// vehicle camera setpoint
			double qc_wb[4] = {ctx->orientation_sp.w, ctx->orientation_sp.x,
					   ctx->orientation_sp.y, ctx->orientation_sp.z};

			double v_w[3];
			{
				// rotate_vector_b_to_w:(q[4],v_b[3])->(v_w[3])
				CASADI_FUNC_ARGS(rotate_vector_b_to_w)

				args[0] = q_wb;
				args[1] = v_b;

				res[0] = v_w;

				CASADI_FUNC_CALL(rotate_vector_b_to_w)
			}

			double nT;       // thrust
			double qr_wb[4]; // attitude setpoint
			{
				// position_control:(thrust_trim,pt_w[3],vt_w[3],at_w[3],
				// qc_wb[4],p_w[3],v_w[3],z_i,dt)->(nT,qr_wb[4],z_i_2)
				CASADI_FUNC_ARGS(position_control)

				args[0] = &thrust_trim;
				args[1] = pt_w;
				args[2] = vt_w;
				args[3] = at_w;
				args[4] = qc_wb;
				args[5] = p_w;
				args[6] = v_w;
				args[7] = z_i;
				args[8] = &dt;

				res[0] = &nT;
				res[1] = qr_wb;
				res[2] = z_i;

				CASADI_FUNC_CALL(position_control)
			}

			double zeta[9]; // se23 error
			{
				// se23_error:(p_w[3],v_b[3],q_wb[4],p_rw[3],v_rw[3],q_r[4])->(zeta[9])
				CASADI_FUNC_ARGS(se23_error)

				args[0] = pt_w;
				args[1] = vt_w;
				args[2] = qr_wb;
				args[3] = p_w;
				args[4] = v_b;
				args[5] = q_wb;

				res[0] = zeta;

				CASADI_FUNC_CALL(se23_error)
			}

			// se23_control:(thrust_trim,kp[3],zeta[9],at_w[3],q_wb[4],z_i,dt)->(nT,z_i_2,u_omega[3],q_sp[4])
			{
				CASADI_FUNC_ARGS(se23_control)

				args[0] = &thrust_trim;
				args[1] = kp;
				args[2] = zeta;
				args[3] = at_w;
				args[4] = q_wb;
				args[5] = &z_i;
				args[6] = &dt;

				res[0] = &nT;
				res[1] = &z_i;
				// res[2] u_omega[3], ignored
				// res[3] q_sp[4], ignored

				CASADI_FUNC_CALL(se23_control)
			}

			bool data_ok = true;
			for (int i = 0; i < 4; i++) {
				if (!isfinite(qr_wb[i])) {
					LOG_ERR("qr_wb[%d] not finite: %10.4f", i, qr_wb[i]);
					data_ok = false;
					break;
				}
			}

			if (!isfinite(nT)) {
				LOG_ERR("nT not finite: %10.4f", nT);
				data_ok = false;
			}

			if (data_ok) {
				stamp_msg(&ctx->attitue_sp.stamp, k_uptime_ticks());
				ctx->attitude_sp.w = qr_wb[0];
				ctx->attitude_sp.x = qr_wb[1];
				ctx->attitude_sp.y = qr_wb[2];
				ctx->attitude_sp.z = qr_wb[3];
				zros_pub_update(&ctx->pub_attitude_sp);

				stamp_msg(&ctx->force_sp.stamp, k_uptime_ticks());
				ctx->force_sp.z = nT;
				zros_pub_update(&ctx->pub_force_sp);
			}
		}
	}
	rdd2_position_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				rdd2_position_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "rdd2_position");
	k_thread_start(tid);
	return 0;
}

static int rdd2_position_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_position, rdd2_position_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_position, &sub_rdd2_position, "rdd2 position commands", NULL);

static int rdd2_position_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_position_sys_init, APPLICATION, 99);

// vi: ts=4 sw=4 et
