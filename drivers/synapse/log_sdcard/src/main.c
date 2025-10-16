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

#define MY_STACK_SIZE 16384
#define MY_PRIORITY   1
//131072
#define BUF_SIZE      131072
#define TOPIC_RATE_HZ 1

#define SUBSCRIBE_TOPIC(topic_name)                                                                \
                                                                                                   \
	ret = zros_sub_init(&ctx->sub_##topic_name, &ctx->node, &topic_##topic_name,               \
			    &ctx->frame.msg, TOPIC_RATE_HZ);                                       \
	if (ret < 0) {                                                                             \
		LOG_ERR("init " #topic_name " failed: %d", ret);                                   \
		return ret;                                                                        \
	}

#define UNSUBSCRIBE_TOPIC(topic_name) zros_sub_fini(&ctx->sub_##topic_name)

#define GET_UPDATE(topic_name, topic_type)                                                         \
	if (zros_sub_update_available(&ctx->sub_##topic_name)) {                                   \
		zros_sub_update(&ctx->sub_##topic_name);                                           \
		ctx->frame.which_msg = synapse_pb_Frame_##topic_type##_tag;                        \
		snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), #topic_name);                 \
		log_sdcard_write_frame(ctx);                                                       \
	}

RING_BUF_DECLARE(rb_sdcard, BUF_SIZE);

LOG_MODULE_REGISTER(log_sdcard, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

/* All Topics
accel_sp - Melody wants
actuators - output
altimeter - want?
angular_velocity_sp - Melody wants
attitude_sp - Melody wants
battery_state - ?
bezier_trajectory - ?
bezier_trajectory_ethernet - ?
clock_offset_ethernet - ?
cmd_vel - ?
cmd_vel_ethernet - ?
imu - output
imu_q31_array - output
input - output
input_ethernet - ?
input_sbus - ?
led_array - ?
magnetic_field - output
moment_ff - ?
nav_sat_fix - Melody Wants
odometry_estimstor - output ("sub_odom")
odometry_ethernet - NEED for ground true
orientation_sp - Melody wants
position_sp - Melody wants
pwm - output
safety - ?
status - ?
velocity_sp - Melody wants
wheel_odometry - ?
*/

struct context {
	// zros node handle
	struct zros_node node;
	// subscriptions
	struct zros_sub sub_accel_sp;
	struct zros_sub sub_actuators;
	// struct zros_sub sub_altimeter;
	struct zros_sub sub_angular_velocity_sp;
	struct zros_sub sub_attitude_sp;
	// struct zros_sub sub_battery_state;
	//struct zros_sub sub_bezier_trajectory;
	//struct zros_sub sub_bezier_trajectory_ethernet;
	struct zros_sub sub_clock_offset_ethernet;
	struct zros_sub sub_cmd_vel;
	struct zros_sub sub_cmd_vel_ethernet;
	//struct zros_sub sub_imu;
	//struct zros_sub sub_imu_q31_array;
	struct zros_sub sub_input;
	struct zros_sub sub_input_ethernet;
	struct zros_sub sub_input_sbus;
	// struct zros_sub sub_led_array;
	struct zros_sub sub_magnetic_field;
	// struct zros_sub sub_moment_ff;
	// struct zros_sub sub_nav_sat_fix;
	struct zros_sub sub_odometry_estimator;
	struct zros_sub sub_odometry_ethernet;
	struct zros_sub sub_orientation_sp;
	struct zros_sub sub_position_sp;
	struct zros_sub sub_pwm;
	struct zros_sub sub_safety;
	struct zros_sub sub_status;
	struct zros_sub sub_velocity_sp;
	// struct zros_sub sub_wheel_odometry,
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
	// Subscriptions
	.sub_accel_sp = {},
	.sub_actuators = {},
	//.sub_altimeter = {},
	//.sub_angular_velocity_ff = {},
	.sub_angular_velocity_sp = {},
	.sub_attitude_sp = {},
	//.sub_battery_state = {},
	//.sub_bezier_trajectory = {},
	//.sub_bezier_trajectory_ethernet = {},
	.sub_clock_offset_ethernet = {},
	.sub_cmd_vel = {},
	.sub_cmd_vel_ethernet = {},
	//.sub_imu = {},
	//.sub_imu_q31_array = {},
	.sub_input = {},
	.sub_input_ethernet = {},
	.sub_input_sbus = {},
	//.sub_led_array = {},
	.sub_magnetic_field = {},
	//.sub_moment_ff = {},
	//.sub_nav_sat_fix = {},
	.sub_odometry_estimator = {},
	.sub_odometry_ethernet = {},
	.sub_orientation_sp = {},
	.sub_position_sp = {},
	.sub_pwm = {},
	.sub_safety = {},
	.sub_status = {},
	.sub_velocity_sp = {},
	// .sub_wheel_odometry = {},
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
	SUBSCRIBE_TOPIC(accel_sp);
	SUBSCRIBE_TOPIC(actuators);
	// SUBSCRIBE_TOPIC(altimeter);
	SUBSCRIBE_TOPIC(angular_velocity_sp);
	SUBSCRIBE_TOPIC(attitude_sp);
	// SUBSCRIBE_TOPIC(battery_state);
	//SUBSCRIBE_TOPIC(bezier_trajectory);
	//SUBSCRIBE_TOPIC(bezier_trajectory_ethernet);
	SUBSCRIBE_TOPIC(clock_offset_ethernet);
	SUBSCRIBE_TOPIC(cmd_vel);
	SUBSCRIBE_TOPIC(cmd_vel_ethernet);
	//SUBSCRIBE_TOPIC(imu);
	//SUBSCRIBE_TOPIC(imu_q31_array);
	SUBSCRIBE_TOPIC(input);
	SUBSCRIBE_TOPIC(input_ethernet);
	SUBSCRIBE_TOPIC(input_sbus);
	// SUBSCRIBE_TOPIC(led_array);
	SUBSCRIBE_TOPIC(magnetic_field);
	// SUBSCRIBE_TOPIC(moment_ff);
	// SUBSCRIBE_TOPIC(nav_sat_fix);
	SUBSCRIBE_TOPIC(odometry_estimator);
	SUBSCRIBE_TOPIC(odometry_ethernet);
	SUBSCRIBE_TOPIC(orientation_sp);
	SUBSCRIBE_TOPIC(position_sp);
	SUBSCRIBE_TOPIC(pwm);
	SUBSCRIBE_TOPIC(safety);
	SUBSCRIBE_TOPIC(status);
	SUBSCRIBE_TOPIC(velocity_sp);
	// SUBSCRIBE_TOPIC(wheel_odometry);

	// SUBSCRIBE_TOPIC(imu);
	// SUBSCRIBE_TOPIC(imu_q31_array);
	// SUBSCRIBE_TOPIC(pwm);
	// SUBSCRIBE_TOPIC(input);
	// SUBSCRIBE_TOPIC(magnetic_field);
	// SUBSCRIBE_TOPIC(odometry_estimator)
	// SUBSCRIBE_TOPIC(actuators);
	// SUBSCRIBE_TOPIC(odometry_ethernet);
	// SUBSCRIBE_TOPIC(position_sp);

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
	UNSUBSCRIBE_TOPIC(accel_sp);
	UNSUBSCRIBE_TOPIC(actuators);
	// UNSUBSCRIBE_TOPIC(altimeter);
	UNSUBSCRIBE_TOPIC(angular_velocity_sp);
	UNSUBSCRIBE_TOPIC(attitude_sp);
	// UNSUBSCRIBE_TOPIC(battery_state);
	//UNSUBSCRIBE_TOPIC(bezier_trajectory);
	//UNSUBSCRIBE_TOPIC(bezier_trajectory_ethernet);
	UNSUBSCRIBE_TOPIC(clock_offset_ethernet);
	UNSUBSCRIBE_TOPIC(cmd_vel);
	UNSUBSCRIBE_TOPIC(cmd_vel_ethernet);
	//UNSUBSCRIBE_TOPIC(imu);
	//UNSUBSCRIBE_TOPIC(imu_q31_array);
	UNSUBSCRIBE_TOPIC(input);
	UNSUBSCRIBE_TOPIC(input_ethernet);
	UNSUBSCRIBE_TOPIC(input_sbus);
	// UNSUBSCRIBE_TOPIC(led_array);
	UNSUBSCRIBE_TOPIC(magnetic_field);
	// UNSUBSCRIBE_TOPIC(moment_ff);
	// UNSUBSCRIBE_TOPIC(nav_sat_fix);
	UNSUBSCRIBE_TOPIC(odometry_estimator);
	UNSUBSCRIBE_TOPIC(odometry_ethernet);
	UNSUBSCRIBE_TOPIC(orientation_sp);
	UNSUBSCRIBE_TOPIC(position_sp);
	UNSUBSCRIBE_TOPIC(pwm);
	UNSUBSCRIBE_TOPIC(safety);
	UNSUBSCRIBE_TOPIC(status);
	UNSUBSCRIBE_TOPIC(velocity_sp);
	// UNSUBSCRIBE_TOPIC(wheel_odometry);

	// UNSUBSCRIBE_TOPIC(imu);
	// UNSUBSCRIBE_TOPIC(imu_q31_array);
	// UNSUBSCRIBE_TOPIC(pwm);
	// UNSUBSCRIBE_TOPIC(input);
	// UNSUBSCRIBE_TOPIC(magnetic_field);
	// UNSUBSCRIBE_TOPIC(odometry_estimator)
	// UNSUBSCRIBE_TOPIC(actuators);
	// UNSUBSCRIBE_TOPIC(odometry_ethernet);
	// UNSUBSCRIBE_TOPIC(position_sp);

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
		 	*zros_sub_get_event(&ctx->sub_accel_sp),
			*zros_sub_get_event(&ctx->sub_actuators),
			//*zros_sub_get_event(&ctx->sub_altimeter),
			*zros_sub_get_event(&ctx->sub_angular_velocity_sp),
			*zros_sub_get_event(&ctx->sub_attitude_sp),
			//*zros_sub_get_event(&ctx->sub_battery_state),
			//*zros_sub_get_event(&ctx->sub_bezier_trajectory),
			//*zros_sub_get_event(&ctx->sub_bezier_trajectory_ethernet),
			*zros_sub_get_event(&ctx->sub_clock_offset_ethernet),
			*zros_sub_get_event(&ctx->sub_cmd_vel),
			*zros_sub_get_event(&ctx->sub_cmd_vel_ethernet),
		 	//*zros_sub_get_event(&ctx->sub_imu),
		 	//*zros_sub_get_event(&ctx->sub_imu_q31_array),
		 	*zros_sub_get_event(&ctx->sub_input),
		 	*zros_sub_get_event(&ctx->sub_input_ethernet),
		 	*zros_sub_get_event(&ctx->sub_input_sbus),
			//*zros_sub_get_event(&ctx->sub_led_array),
			*zros_sub_get_event(&ctx->sub_magnetic_field),
		 	//*zros_sub_get_event(&ctx->sub_moment_ff),
		 	//*zros_sub_get_event(&ctx->sub_nav_sat_fix),
		 	*zros_sub_get_event(&ctx->sub_odometry_estimator),
		 	*zros_sub_get_event(&ctx->sub_odometry_ethernet),
		 	*zros_sub_get_event(&ctx->sub_orientation_sp),
		 	*zros_sub_get_event(&ctx->sub_position_sp),
		 	*zros_sub_get_event(&ctx->sub_pwm),
		 	*zros_sub_get_event(&ctx->sub_safety),
		 	*zros_sub_get_event(&ctx->sub_status),
		 	*zros_sub_get_event(&ctx->sub_velocity_sp),
		 	//*zros_sub_get_event(&ctx->sub_wheel_odometry),
		};

		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(2000));
		if (rc != 0) {
			LOG_DBG("poll timeout");
		}

		perf_counter_update(&ctx->perf);

		// check for updates
		GET_UPDATE(accel_sp, vector3);
		GET_UPDATE(actuators, actuators);
		// GET_UPDATE(altimeter, altimeter);
		GET_UPDATE(angular_velocity_sp, vector3);
		GET_UPDATE(attitude_sp, quaternion);
		// GET_UPDATE(battery_state, battery_state);
		//GET_UPDATE(bezier_trajectory, bezier_trajectory);
		//GET_UPDATE(bezier_trajectory_ethernet, bezier_trajectory);
		GET_UPDATE(clock_offset_ethernet, clock_offset);
		GET_UPDATE(cmd_vel, twist);
		GET_UPDATE(cmd_vel_ethernet, twist);
		//GET_UPDATE(imu, imu);
		//GET_UPDATE(imu_q31_array, imu_q31_array);
		GET_UPDATE(input, input);
		// GET_UPDATE(input_ethernet, input);
		// GET_UPDATE(input_sbus, input);
		// GET_UPDATE(led_array, led_array);
		GET_UPDATE(magnetic_field, magnetic_field);
		// GET_UPDATE(moment_ff, vector3);
		// GET_UPDATE(nav_sat_fix, nav_sat_fix);
		GET_UPDATE(odometry_estimator, odometry);
		GET_UPDATE(odometry_ethernet, odometry);
		// GET_UPDATE(orientation_sp, vector3); // Why?
		GET_UPDATE(position_sp, vector3);
		GET_UPDATE(pwm, pwm);
		GET_UPDATE(safety, safety);
		GET_UPDATE(status, status);
		GET_UPDATE(velocity_sp, vector3);
		//  GET_UPDATE(wheel_odometry, wheel_odometry);
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
