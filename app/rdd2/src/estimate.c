/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/perf_counter.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>

#include "app/rdd2/casadi/rdd2.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(rdd2_estimate, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
	struct zros_node node;
	synapse_pb_Odometry odometry_ethernet;
	synapse_pb_Imu imu;
	synapse_pb_Odometry odometry;
	struct zros_sub sub_odometry_ethernet, sub_imu, sub_mag;
	struct zros_pub pub_odometry;
	double x[3];
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	struct perf_counter perf;
	synapse_pb_MagneticField mag;
};

// private initialization
static struct context g_ctx = {
	.node = {},
	.odometry_ethernet = synapse_pb_Odometry_init_default,
	.imu = synapse_pb_Imu_init_default,
	.odometry =
		{
			.child_frame_id = "base_link",
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.frame_id = "odom",
			.has_pose = true,
			.has_twist = true,
			.pose.has_position = true,
			.pose.has_orientation = true,
			.twist.has_angular = true,
			.twist.has_linear = true,
		},
	.sub_odometry_ethernet = {},
	.sub_imu = {},
	.sub_mag = {},
	.pub_odometry = {},
	.x = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.perf = {},
	.mag = {},
};

static void rdd2_estimate_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_estimate");
	zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 300);
	zros_sub_init(&ctx->sub_mag, &ctx->node, &topic_magnetic_field, &ctx->mag, 300);
	zros_sub_init(&ctx->sub_odometry_ethernet, &ctx->node, &topic_odometry_ethernet,
		      &ctx->odometry_ethernet, 10);
	zros_pub_init(&ctx->pub_odometry, &ctx->node, &topic_odometry_estimator, &ctx->odometry);
	perf_counter_init(&ctx->perf, "estimator imu", 1.0 / 100);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_estimate_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_imu);
	zros_sub_fini(&ctx->sub_mag);
	zros_sub_fini(&ctx->sub_odometry_ethernet);
	zros_pub_fini(&ctx->pub_odometry);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void rdd2_estimate_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int rc = 0;

	// LOG_DBG("started");
	rdd2_estimate_init(ctx);

	// variables
	struct k_poll_event events[1] = {};

	// wait for imu
	LOG_DBG("waiting for imu");
	events[0] = *zros_sub_get_event(&ctx->sub_imu);
	rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
	if (rc != 0) {
		LOG_DBG("did not receive imu");
		return;
	}
	zros_sub_update(&ctx->sub_imu);

	double dt = 0;
	int64_t ticks_last = k_uptime_ticks();

	// estimator states
	double x[10] = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
	double q[4] = {1, 0, 0, 0};

	// estimator covariance
	double P[36] = {1e-2, 0, 0, 0,    0, 0, 0, 1e-2, 0, 0, 0,    0, 0, 0, 1e-2, 0, 0, 0,
			0,    0, 0, 1e-2, 0, 0, 0, 0,    0, 0, 1e-2, 0, 0, 0, 0,    0, 0, 1e-2};

	// poll on imu
	events[0] = *zros_sub_get_event(&ctx->sub_imu);
	// int j = 0;

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

		// j += 1;

		// poll for imu
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG("not receiving imu");
			continue;
		}

		if (zros_sub_update_available(&ctx->sub_imu)) {
			zros_sub_update(&ctx->sub_imu);
			perf_counter_update(&ctx->perf);
		}

		if (zros_sub_update_available(&ctx->sub_mag)) {
			zros_sub_update(&ctx->sub_mag);
		}

		/*
		if (j % 100 == 0) {
		    int offset = 0;
		    static char buf[1024];
		    int n = 1024;
		    offset += snprintf(buf + offset, n - offset, "x: ");
		    for (int i=0; i<10;i++) {
			offset += snprintf(buf + offset, n - offset, " %6.2f", x[i]);
		    }
		    LOG_INF("%s", buf);
		}
		*/

		if (zros_sub_update_available(&ctx->sub_odometry_ethernet)) {
			// LOG_INF("correct offboard odometry");
			zros_sub_update(&ctx->sub_odometry_ethernet);

#if defined(CONFIG_CEREBRI_RDD2_ESTIMATE_ODOMETRY_ETHERNET)
			__ASSERT(fabs((ctx->odometry_ethernet.pose.orientation.w *
					       ctx->odometry_ethernet.pose.orientation.w +
				       ctx->odometry_ethernet.pose.orientation.x *
					       ctx->odometry_ethernet.pose.orientation.x +
				       ctx->odometry_ethernet.pose.orientation.y *
					       ctx->odometry_ethernet.pose.orientation.y +
				       ctx->odometry_ethernet.pose.orientation.z *
					       ctx->odometry_ethernet.pose.orientation.z) -
				      1) < 1e-2,
				 "quaternion normal error");

			// use offboard odometry to reset position
			x[0] = ctx->odometry_ethernet.pose.position.x;
			x[1] = ctx->odometry_ethernet.pose.position.y;
			x[2] = ctx->odometry_ethernet.pose.position.z;

			// use offboard odometry to reset velocity
			x[3] = ctx->odometry_ethernet.twist.linear.x;
			x[4] = ctx->odometry_ethernet.twist.linear.y;
			x[5] = ctx->odometry_ethernet.twist.linear.z;

			// use offboard odometry to reset orientation
			x[6] = ctx->odometry_ethernet.pose.orientation.w;
			x[7] = ctx->odometry_ethernet.pose.orientation.x;
			x[8] = ctx->odometry_ethernet.pose.orientation.y;
			x[9] = ctx->odometry_ethernet.pose.orientation.z;
#endif
		}

		// calculate dt
		int64_t ticks_now = k_uptime_ticks();
		dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		ticks_last = ticks_now;
		if (dt <= 0 || dt > 0.5) {
			LOG_WRN("imu update rate too low");
			continue;
		}

		{
			CASADI_FUNC_ARGS(strapdown_ins_propagate)
			/* strapdown_ins_propagate:(x0[10],a_b[3],omega_b[3],g,dt)->(x1[10]) */
			const double g = 9.8;
			double a_b[3] = {ctx->imu.linear_acceleration.x,
					 ctx->imu.linear_acceleration.y,
					 ctx->imu.linear_acceleration.z};
			double omega_b[3] = {ctx->imu.angular_velocity.x,
					     ctx->imu.angular_velocity.y,
					     ctx->imu.angular_velocity.z};
			args[0] = x;
			args[1] = a_b;
			args[2] = omega_b;
			args[3] = &g;
			args[4] = &dt;
			res[0] = x;
			CASADI_FUNC_CALL(strapdown_ins_propagate)
		}

		// {
		// 	CASADI_FUNC_ARGS(position_correction)

		// 	double gps[3] = {ctx->odometry_ethernet.pose.position.x,
		// 			 ctx->odometry_ethernet.pose.position.y,
		// 			 ctx->odometry_ethernet.pose.position.z};

		// 	args[0] = x;
		// 	args[1] = gps;
		// 	args[2] = &dt;
		// 	args[3] = P;

		// 	res[0] = x;
		// 	res[1] = P;

		// 	CASADI_FUNC_CALL(position_correction)
		// }

		/*
		f_att_estimator = ca.Function(
	"attitude_estimator",
	[q0, mag, mag_decl, gyro, accel, dt],
	[q1.param],
	["q", "mag", "mag_decl", "gyro", "accel", "dt"],
	["q1"],
	)*/
		{
			CASADI_FUNC_ARGS(attitude_estimator)

			double a_b[3] = {ctx->imu.linear_acceleration.x,
					 ctx->imu.linear_acceleration.y,
					 ctx->imu.linear_acceleration.z};
			double omega_b[3] = {ctx->imu.angular_velocity.x,
					     ctx->imu.angular_velocity.y,
					     ctx->imu.angular_velocity.z};
 
			double mag[3] = {ctx->mag.magnetic_field.x, ctx->mag.magnetic_field.y,
					 ctx->mag.magnetic_field.z};
			//const double decl_WL = -6.66/180 * M_PI + 0.12;
			const double decl_WL = 0;
			//double debug[1];
			q[0] = x[6];
			q[1] = x[7];
			q[2] = x[8];
			q[3] = x[9];

			args[0] = q;
			args[1] = mag;
			args[2] = &decl_WL;
			args[3] = omega_b;
			args[4] = a_b;
			args[5] = &dt;
			res[0] = q;
			//res[1] = debug;

			CASADI_FUNC_CALL(attitude_estimator)

		}
		

			//LOG_ERR("mag_err: {%.2f}", debug[0]);
			//LOG_INF("mag: x {%.2f} y, {%.2f} z {%.2f}", mag[0], mag[1], mag[2]);
			//LOG_INF("q: {%.2f} {%.2f} {%.2f} {%.2f}", q[0], q[1], q[2], q[3]);
			x[6] = q[0];
			x[7] = q[1];
			x[8] = q[2];
			x[9] = q[3];

			P[0] = P[0]; // temp line


		bool data_ok = true;
		for (int i = 0; i < 10; i++) {
			if (!isfinite(x[i])) {
				LOG_ERR("x[%d] is not finite", i);
				// TODO reinitialize
				x[i] = 0;
				data_ok = false;
				break;
			}
		}

		// publish odometry
		if (data_ok) {
			stamp_msg(&ctx->odometry.stamp, k_uptime_ticks());
			ctx->odometry.pose.position.x = x[0];
			ctx->odometry.pose.position.y = x[1];
			ctx->odometry.pose.position.z = x[2];
			ctx->odometry.twist.linear.x = x[3];
			ctx->odometry.twist.linear.y = x[4];
			ctx->odometry.twist.linear.z = x[5];
			ctx->odometry.pose.orientation.w = x[6];
			ctx->odometry.pose.orientation.x = x[7];
			ctx->odometry.pose.orientation.y = x[8];
			ctx->odometry.pose.orientation.z = x[9];
			ctx->odometry.twist.angular.x = ctx->imu.angular_velocity.x;
			ctx->odometry.twist.angular.y = ctx->imu.angular_velocity.y;
			ctx->odometry.twist.angular.z = ctx->imu.angular_velocity.z;

			// check quaternion normal
			__ASSERT(fabs((ctx->odometry.pose.orientation.w *
					       ctx->odometry.pose.orientation.w +
				       ctx->odometry.pose.orientation.x *
					       ctx->odometry.pose.orientation.x +
				       ctx->odometry.pose.orientation.y *
					       ctx->odometry.pose.orientation.y +
				       ctx->odometry.pose.orientation.z *
					       ctx->odometry.pose.orientation.z) -
				      1) < 1e-2,
				 "quaternion normal error");
			zros_pub_update(&ctx->pub_odometry);
		}
	}

	rdd2_estimate_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				rdd2_estimate_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "rdd2_estimate");
	k_thread_start(tid);
	return 0;
}

static int rdd2_estimate_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_estimate, rdd2_estimate_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_estimate, &sub_rdd2_estimate, "rdd2 estimate commands", NULL);

static int rdd2_estimate_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_estimate_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
