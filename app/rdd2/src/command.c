/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include "command.h"
#include <cerebri/core/log_utils.h>

#define MY_STACK_SIZE 16384
#define MY_PRIORITY   4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

CEREBRI_NODE_LOG_INIT(rdd2_command, LOG_LEVEL_WRN);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

static struct context g_ctx = {
	.node = {},
	.input = synapse_pb_Input_init_default,
	.attitude_sp = synapse_pb_Quaternion_init_default,
	.angular_velocity_ff = synapse_pb_Vector3_init_default,
	.force_sp = synapse_pb_Vector3_init_default,
	.bezier_trajectory = synapse_pb_BezierTrajectory_init_default,
	.status = synapse_pb_Status_init_default,
	.last_status = synapse_pb_Status_init_default,
	.velocity_sp = synapse_pb_Vector3_init_default,
	.accel_sp = synapse_pb_Vector3_init_default,
	.moment_ff = synapse_pb_Vector3_init_default,
	.orientation_sp = synapse_pb_Quaternion_init_default,
	.position_sp =
		{
			.has_stamp = true,
			.x = 0,
			.y = 0,
			.z = 0,
		},
	.cmd_vel = synapse_pb_Twist_init_default,
	.sub_input_ethernet = {},
	.sub_input_sbus = {},
	.sub_status = {},
	.sub_bezier_trajectory_ethernet = {},
	.sub_clock_offset_ethernet = {},
	.sub_odometry_estimator = {},
	.sub_cmd_vel_ethernet = {},
	.pub_attitude_sp = {},
	.pub_angular_velocity_ff = {},
	.pub_force_sp = {},
	.pub_velocity_sp = {},
	.pub_accel_sp = {},
	.pub_moment_ff = {},
	.pub_orientation_sp = {},
	.pub_position_sp = {},
	.pub_input = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.psi_sp = 0,
	.input_aetr = {0, 0, 0, 0},
	.q = {1, 0, 0, 0},
	.dt = 0,
	.thrust_trim = CONFIG_CEREBRI_RDD2_THRUST_TRIM * 1e-3,
	.thrust_delta = CONFIG_CEREBRI_RDD2_THRUST_DELTA * 1e-3,
};

static void rdd2_command_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_command");
	zros_sub_init(&ctx->sub_input_ethernet, &ctx->node, &topic_input_ethernet, &ctx->input,
		      200);
	zros_sub_init(&ctx->sub_input_sbus, &ctx->node, &topic_input_sbus, &ctx->input, 200);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_bezier_trajectory_ethernet, &ctx->node,
		      &topic_bezier_trajectory_ethernet, &ctx->bezier_trajectory, 10);
	zros_sub_init(&ctx->sub_clock_offset_ethernet, &ctx->node, &topic_clock_offset_ethernet,
		      &ctx->clock_offset, 10);
	zros_sub_init(&ctx->sub_odometry_estimator, &ctx->node, &topic_odometry_estimator,
		      &ctx->odometry_estimator, 10);
	zros_sub_init(&ctx->sub_cmd_vel_ethernet, &ctx->node, &topic_cmd_vel_ethernet,
		      &ctx->cmd_vel, 10);
	zros_pub_init(&ctx->pub_attitude_sp, &ctx->node, &topic_attitude_sp, &ctx->attitude_sp);
	zros_pub_init(&ctx->pub_angular_velocity_ff, &ctx->node, &topic_angular_velocity_ff,
		      &ctx->angular_velocity_ff);
	zros_pub_init(&ctx->pub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp);
	zros_pub_init(&ctx->pub_velocity_sp, &ctx->node, &topic_velocity_sp, &ctx->velocity_sp);
	zros_pub_init(&ctx->pub_accel_sp, &ctx->node, &topic_accel_sp, &ctx->accel_sp);
	zros_pub_init(&ctx->pub_moment_ff, &ctx->node, &topic_moment_ff, &ctx->moment_ff);
	zros_pub_init(&ctx->pub_orientation_sp, &ctx->node, &topic_orientation_sp,
		      &ctx->orientation_sp);
	zros_pub_init(&ctx->pub_position_sp, &ctx->node, &topic_position_sp, &ctx->position_sp);
	zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input, &ctx->input);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_command_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_input_ethernet);
	zros_sub_fini(&ctx->sub_input_sbus);
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_bezier_trajectory_ethernet);
	zros_sub_fini(&ctx->sub_odometry_estimator);
	zros_sub_fini(&ctx->sub_cmd_vel_ethernet);
	zros_pub_fini(&ctx->pub_attitude_sp);
	zros_pub_fini(&ctx->pub_angular_velocity_ff);
	zros_pub_fini(&ctx->pub_velocity_sp);
	zros_pub_fini(&ctx->pub_accel_sp);
	zros_pub_fini(&ctx->pub_force_sp);
	zros_pub_fini(&ctx->pub_orientation_sp);
	zros_pub_fini(&ctx->pub_position_sp);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void rdd2_command_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	rdd2_command_init(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_input_ethernet),
		*zros_sub_get_event(&ctx->sub_input_sbus),
		*zros_sub_get_event(&ctx->sub_cmd_vel_ethernet),
		*zros_sub_get_event(&ctx->sub_bezier_trajectory_ethernet),
	};

	int64_t ticks_last = k_uptime_ticks();

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		// wait for input event, publish at 1 Hz regardless
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			// LOG_DBG("not receiving input");
			ctx->status.mode = synapse_pb_Status_Mode_MODE_ATTITUDE;
			for (int i = 0; i < ctx->input.channel_count; i++) {
				ctx->input.channel[i] = 0;
			}
			ctx->cmd_vel.linear.x = 0;
			ctx->cmd_vel.linear.y = 0;
			ctx->cmd_vel.linear.z = 0;
			ctx->cmd_vel.angular.x = 0;
			ctx->cmd_vel.angular.y = 0;
			ctx->cmd_vel.angular.z = 0;
			ctx->cmd_vel.has_linear = true;
			ctx->cmd_vel.has_angular = true;
		}

		if (zros_sub_update_available(&ctx->sub_status)) {
			// record last status
			ctx->last_status = ctx->status;
			zros_sub_update(&ctx->sub_status);
		}

		if (zros_sub_update_available(&ctx->sub_clock_offset_ethernet)) {
			zros_sub_update(&ctx->sub_clock_offset_ethernet);
		}

		// prioritize onboard sbus input
		if (zros_sub_update_available(&ctx->sub_input_sbus)) {
			zros_sub_update(&ctx->sub_input_sbus);
			zros_pub_update(&ctx->pub_input);
			ctx->status.input_source =
				synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL;
		} else if (zros_sub_update_available(&ctx->sub_input_ethernet)) {
			zros_sub_update(&ctx->sub_input_ethernet);
			zros_pub_update(&ctx->pub_input);
			ctx->status.input_source =
				synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET;
		}

		zros_sub_update(&ctx->sub_bezier_trajectory_ethernet);
		zros_sub_update(&ctx->sub_odometry_estimator);
		zros_sub_update(&ctx->sub_cmd_vel_ethernet);

		// calculate dt
		int64_t ticks_now = k_uptime_ticks();
		ctx->dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		ticks_last = ticks_now;
		if (ctx->dt < 0 || ctx->dt > 0.5) {
			LOG_DBG("input update rate too low");
			continue;
		}

		// input data
		ctx->input_aetr[0] = ctx->input.channel[CH_RIGHT_STICK_RIGHT];
		ctx->input_aetr[1] = ctx->input.channel[CH_RIGHT_STICK_UP];
		ctx->input_aetr[2] = ctx->input.channel[CH_LEFT_STICK_UP];
		ctx->input_aetr[3] = -ctx->input.channel[CH_LEFT_STICK_RIGHT];

		// estimated attitude quaternion
		ctx->q[0] = ctx->odometry_estimator.pose.orientation.w;
		ctx->q[1] = ctx->odometry_estimator.pose.orientation.x;
		ctx->q[2] = ctx->odometry_estimator.pose.orientation.y;
		ctx->q[3] = ctx->odometry_estimator.pose.orientation.z;

		// handle joy based on mode
		if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ATTITUDE_RATE) {
			rdd2_mode_attitude_rate(ctx);
		} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ATTITUDE) {
			rdd2_mode_attitude(ctx);
		} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_VELOCITY) {
			rdd2_mode_velocity(ctx);
		} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_BEZIER) {
			rdd2_mode_bezier(ctx);
		} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_UNKNOWN) {
			// LOG_ERR("unknown mode");
		}
	}
	rdd2_command_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      rdd2_command_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "rdd2_command");
	k_thread_start(tid);
	return 0;
}

static int rdd2_command_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_command, rdd2_command_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_command, &sub_rdd2_command, "rdd2 command arguments", NULL);

static int rdd2_command_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_command_sys_init, APPLICATION, 99);

// vi: ts=4 sw=4 et
