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
#define TOPIC_RATE_HZ 100
#define MAX_ENC_SIZE  8192

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
		harden_pb_frame(&ctx->frame);                                                      \
		ctx->frame.which_msg = synapse_pb_Frame_##topic_type##_tag;                        \
		snprintf(ctx->frame.topic, sizeof(ctx->frame.topic), #topic_name);                 \
		log_sdcard_write_frame(ctx);                                                       \
	}

BUILD_ASSERT(BUF_SIZE <= RING_BUFFER_MAX_SIZE, RING_BUFFER_SIZE_ASSERT_MSG);
static uint8_t _ring_buffer_data_rb_sdcard[BUF_SIZE];
struct ring_buf rb_sdcard = RING_BUF_INIT(_ring_buffer_data_rb_sdcard, BUF_SIZE);

LOG_MODULE_REGISTER(log_sdcard, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	// zros node handle
	struct zros_node node;
	// subscriptions
	struct zros_sub sub_accel_sp;
	struct zros_sub sub_actuators;
	struct zros_sub sub_altimeter;
	// struct zros_sub sub_angular_velocity_ff;
	struct zros_sub sub_angular_velocity_sp;
	struct zros_sub sub_attitude_sp;
	struct zros_sub sub_battery_state;
	struct zros_sub sub_bezier_trajectory;
	// struct zros_sub sub_bezier_trajectory_ethernet;
	struct zros_sub sub_clock_offset_ethernet;
	struct zros_sub sub_cmd_vel;
	struct zros_sub sub_cmd_vel_ethernet;
	struct zros_sub sub_imu;
	// struct zros_sub sub_imu_q31_array;
	struct zros_sub sub_input;
	struct zros_sub sub_input_ethernet;
	struct zros_sub sub_input_rc;
	struct zros_sub sub_led_array;
	struct zros_sub sub_magnetic_field;
	struct zros_sub sub_moment_ff;
	struct zros_sub sub_nav_sat_fix;
	struct zros_sub sub_odometry_estimator;
	struct zros_sub sub_odometry_ethernet;
	struct zros_sub sub_orientation_sp;
	struct zros_sub sub_position_sp;
	struct zros_sub sub_pwm;
	struct zros_sub sub_safety;
	struct zros_sub sub_status;
	struct zros_sub sub_velocity_sp;
	struct zros_sub sub_wheel_odometry;
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
	.sub_altimeter = {},
	//.sub_angular_velocity_ff = {},
	.sub_angular_velocity_sp = {},
	.sub_attitude_sp = {},
	.sub_battery_state = {},
	.sub_bezier_trajectory = {},
	//.sub_bezier_trajectory_ethernet = {},
	.sub_clock_offset_ethernet = {},
	.sub_cmd_vel = {},
	.sub_cmd_vel_ethernet = {},
	.sub_imu = {},
	//.sub_imu_q31_array = {},
	.sub_input = {},
	.sub_input_ethernet = {},
	.sub_input_rc = {},
	.sub_led_array = {},
	.sub_magnetic_field = {},
	.sub_moment_ff = {},
	.sub_nav_sat_fix = {},
	.sub_odometry_estimator = {},
	.sub_odometry_ethernet = {},
	.sub_orientation_sp = {},
	.sub_position_sp = {},
	.sub_pwm = {},
	.sub_safety = {},
	.sub_status = {},
	.sub_velocity_sp = {},
	.sub_wheel_odometry = {},
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
	SUBSCRIBE_TOPIC(altimeter);
	SUBSCRIBE_TOPIC(angular_velocity_sp);
	SUBSCRIBE_TOPIC(attitude_sp);
	SUBSCRIBE_TOPIC(battery_state);
	SUBSCRIBE_TOPIC(bezier_trajectory);
	// SUBSCRIBE_TOPIC(bezier_trajectory_ethernet);
	SUBSCRIBE_TOPIC(clock_offset_ethernet);
	SUBSCRIBE_TOPIC(cmd_vel);
	SUBSCRIBE_TOPIC(cmd_vel_ethernet);
	SUBSCRIBE_TOPIC(imu);
	// SUBSCRIBE_TOPIC(imu_q31_array);
	SUBSCRIBE_TOPIC(input);
	SUBSCRIBE_TOPIC(input_ethernet);
	SUBSCRIBE_TOPIC(input_rc);
	SUBSCRIBE_TOPIC(led_array);
	SUBSCRIBE_TOPIC(magnetic_field);
	SUBSCRIBE_TOPIC(moment_ff);
	SUBSCRIBE_TOPIC(nav_sat_fix);
	SUBSCRIBE_TOPIC(odometry_estimator);
	SUBSCRIBE_TOPIC(odometry_ethernet);
	SUBSCRIBE_TOPIC(orientation_sp);
	SUBSCRIBE_TOPIC(position_sp);
	SUBSCRIBE_TOPIC(pwm);
	SUBSCRIBE_TOPIC(safety);
	SUBSCRIBE_TOPIC(status);
	SUBSCRIBE_TOPIC(velocity_sp);
	SUBSCRIBE_TOPIC(wheel_odometry);

	k_sem_take(&ctx->running, K_FOREVER);

	LOG_INF("reader init");
	return ret;
};

static int log_sdcard_fini(struct context *ctx)
{
	int ret = 0;

	// close subscriptions
	UNSUBSCRIBE_TOPIC(accel_sp);
	UNSUBSCRIBE_TOPIC(actuators);
	UNSUBSCRIBE_TOPIC(altimeter);
	UNSUBSCRIBE_TOPIC(angular_velocity_sp);
	UNSUBSCRIBE_TOPIC(attitude_sp);
	UNSUBSCRIBE_TOPIC(battery_state);
	UNSUBSCRIBE_TOPIC(bezier_trajectory);
	// UNSUBSCRIBE_TOPIC(bezier_trajectory_ethernet);
	UNSUBSCRIBE_TOPIC(clock_offset_ethernet);
	UNSUBSCRIBE_TOPIC(cmd_vel);
	UNSUBSCRIBE_TOPIC(cmd_vel_ethernet);
	UNSUBSCRIBE_TOPIC(imu);
	// UNSUBSCRIBE_TOPIC(imu_q31_array);
	UNSUBSCRIBE_TOPIC(input);
	UNSUBSCRIBE_TOPIC(input_ethernet);
	UNSUBSCRIBE_TOPIC(input_rc);
	UNSUBSCRIBE_TOPIC(led_array);
	UNSUBSCRIBE_TOPIC(magnetic_field);
	UNSUBSCRIBE_TOPIC(moment_ff);
	UNSUBSCRIBE_TOPIC(nav_sat_fix);
	UNSUBSCRIBE_TOPIC(odometry_estimator);
	UNSUBSCRIBE_TOPIC(odometry_ethernet);
	UNSUBSCRIBE_TOPIC(orientation_sp);
	UNSUBSCRIBE_TOPIC(position_sp);
	UNSUBSCRIBE_TOPIC(pwm);
	UNSUBSCRIBE_TOPIC(safety);
	UNSUBSCRIBE_TOPIC(status);
	UNSUBSCRIBE_TOPIC(velocity_sp);
	UNSUBSCRIBE_TOPIC(wheel_odometry);

	zros_node_fini(&ctx->node);

	k_sem_give(&ctx->running);
	LOG_INF("reader fini");
	return ret;
};

/* Streaming write callback writing to Zephyr ring buffer in chunks */
static bool rb_stream_write(pb_ostream_t *stream, const pb_byte_t *buf, size_t count)
{
	struct ring_buf *rb = (struct ring_buf *)stream->state;

	size_t written = 0;
	while (written < count) {
		uint8_t *dst;
		size_t to_write = count - written;

		/* Claim as much as possible for this chunk */
		size_t claim = ring_buf_put_claim(rb, &dst, to_write);
		if (claim == 0) {
			/* No contiguous space at this moment
			 * Fail: this will abort the encode.
			 */
			PB_RETURN_ERROR(stream, "ring buffer full");
			return false;
		}

		memcpy(dst, buf + written, claim);
		ring_buf_put_finish(rb, claim);
		written += claim;
	}

	return true;
}

static void log_sdcard_write_frame(struct context *ctx)
{
	uint8_t *buf;
	uint32_t claimed_size = ring_buf_put_claim(&rb_sdcard, &buf, MAX_ENC_SIZE);

	if (claimed_size < MAX_ENC_SIZE) {
		/* No data written, release claim */
		ring_buf_put_finish(&rb_sdcard, 0);

		/* Build a nanopb stream that writes via our callback */
		pb_ostream_t stream = {.callback = rb_stream_write,
				       .state = &rb_sdcard,
				       .max_size = SIZE_MAX,
				       .bytes_written = 0};

		if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, &ctx->frame,
				  PB_ENCODE_DELIMITED)) {
			/* Note: Partial data may have been committed to the ring buffer */
			LOG_ERR_RATELIMIT_RATE(1000, "encoding failed: %s", PB_GET_ERROR(&stream));
		}
	} else {
		pb_ostream_t stream = pb_ostream_from_buffer(buf, claimed_size);
		if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, &ctx->frame,
				  PB_ENCODE_DELIMITED)) {
			LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
			ring_buf_put_finish(&rb_sdcard, 0);
		} else {
			ring_buf_put_finish(&rb_sdcard, stream.bytes_written);
		}
	}
}

static void harden_pb_frame(synapse_pb_Frame *frame)
{
	memset(&frame->cb_msg, 0, sizeof(frame->cb_msg));
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
		GET_UPDATE(altimeter, altimeter);
		GET_UPDATE(angular_velocity_sp, vector3);
		GET_UPDATE(attitude_sp, quaternion);
		GET_UPDATE(battery_state, battery_state);
		GET_UPDATE(bezier_trajectory, bezier_trajectory);
		// GET_UPDATE(bezier_trajectory_ethernet, bezier_trajectory);
		GET_UPDATE(clock_offset_ethernet, clock_offset);
		GET_UPDATE(cmd_vel, twist);
		GET_UPDATE(cmd_vel_ethernet, twist);
		GET_UPDATE(imu, imu);
		// GET_UPDATE(imu_q31_array, imu_q31_array);
		GET_UPDATE(input, input);
		GET_UPDATE(input_ethernet, input);
		GET_UPDATE(input_rc, input);
		GET_UPDATE(led_array, led_array);
		GET_UPDATE(magnetic_field, magnetic_field);
		GET_UPDATE(moment_ff, vector3);
		GET_UPDATE(nav_sat_fix, nav_sat_fix);
		GET_UPDATE(odometry_estimator, odometry);
		GET_UPDATE(odometry_ethernet, odometry);
		GET_UPDATE(orientation_sp, vector3);
		GET_UPDATE(position_sp, vector3);
		GET_UPDATE(pwm, pwm);
		GET_UPDATE(safety, safety);
		GET_UPDATE(status, status);
		GET_UPDATE(velocity_sp, vector3);
		GET_UPDATE(wheel_odometry, wheel_odometry);
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

SYS_INIT(log_sdcard_sys_init, APPLICATION, 99);

// vi: ts=4 sw=4 et
