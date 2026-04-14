/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flight_mode.h"
#include "imu_latency_stats.h"
#include "topic_bus.h"
#include "topic_flatbuffer.h"

#include <errno.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/shell/shell.h>

#include <zros/zros_shell.h>

#define TOPIC_VEC3_FMT "[%9.5f %9.5f %9.5f]"
#define TOPIC_RATE_FMT "[%9.6f %9.6f %9.6f]"

static void rdd2_topic_shell_print_latency_stats(const struct shell *sh)
{
	struct rdd2_imu_latency_stats_snapshot snapshot;

	if (!rdd2_imu_latency_stats_get(&snapshot)) {
		shell_print(sh, "latency us n=0");
		return;
	}

	shell_print(sh, "latency us last=%4lu mean=%7.2f std=%7.2f min=%4lu max=%4lu n=%llu",
		    (unsigned long)snapshot.last_us, (double)snapshot.mean_us,
		    (double)snapshot.stddev_us, (unsigned long)snapshot.min_us,
		    (unsigned long)snapshot.max_us, (unsigned long long)snapshot.sample_count);
}

static void rdd2_topic_shell_format_flight_state(const struct shell *sh,
						 const struct zros_topic *topic, const void *msg,
						 size_t msg_size)
{
	synapse_topic_Vec3f_t gyro = {0};
	synapse_topic_Vec3f_t accel = {0};
	synapse_topic_RcChannels16_t rc = {0};
	synapse_topic_ControlStatus_t status = {0};
	synapse_topic_AttitudeEuler_t attitude = {0};
	synapse_topic_AttitudeEuler_t attitude_desired = {0};
	synapse_topic_RateTriplet_t rate_desired = {0};
	synapse_topic_RateTriplet_t rate_cmd = {0};
	uint32_t main_loop_latency_us = 0U;
	int64_t now_ms = k_uptime_get();
	int64_t rc_age_ms;

	ARG_UNUSED(topic);

	if (!rdd2_topic_fb_unpack_flight_state(msg, msg_size, &gyro, &accel, &rc, &status,
					       &attitude, &attitude_desired, &rate_desired,
					       &rate_cmd, &main_loop_latency_us)) {
		shell_error(sh, "flight_state: invalid snapshot");
		return;
	}

	rc_age_ms = status.rc_valid ? (now_ms - status.rc_stamp_ms) : -1;
	shell_print(sh,
		    "status mode=%-10s armed=%1d imu_ok=%1d rc_valid=%1d rc_stale=%1d "
		    "arm_switch=%1d rc_age_ms=%6lld lq=%3u throttle_us=%4ld imu_to_motor_us=%4lu",
		    rdd2_flight_mode_name((enum rdd2_flight_mode)status.flight_mode),
		    status.armed ? 1 : 0, status.imu_ok ? 1 : 0, status.rc_valid ? 1 : 0,
		    status.rc_stale ? 1 : 0, status.arm_switch ? 1 : 0, (long long)rc_age_ms,
		    (unsigned int)status.rc_link_quality, (long)status.throttle_us,
		    (unsigned long)main_loop_latency_us);
	rdd2_topic_shell_print_latency_stats(sh);
	shell_print(sh, "imu gyro_rad_s=" TOPIC_VEC3_FMT " accel_m_s2=" TOPIC_VEC3_FMT,
		    (double)gyro.x, (double)gyro.y, (double)gyro.z, (double)accel.x,
		    (double)accel.y, (double)accel.z);
	shell_print(sh, "attitude     rpy=" TOPIC_RATE_FMT " attitude_sp  rpy=" TOPIC_RATE_FMT,
		    (double)attitude.roll, (double)attitude.pitch, (double)attitude.yaw,
		    (double)attitude_desired.roll, (double)attitude_desired.pitch,
		    (double)attitude_desired.yaw);
	shell_print(sh, "rate_desired xyz=" TOPIC_RATE_FMT " rate_cmd     xyz=" TOPIC_RATE_FMT,
		    (double)rate_desired.roll, (double)rate_desired.pitch, (double)rate_desired.yaw,
		    (double)rate_cmd.roll, (double)rate_cmd.pitch, (double)rate_cmd.yaw);
	shell_print(sh, "rc ch0-7=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]", (long)rc.ch0,
		    (long)rc.ch1, (long)rc.ch2, (long)rc.ch3, (long)rc.ch4, (long)rc.ch5,
		    (long)rc.ch6, (long)rc.ch7);
}

static void rdd2_topic_shell_format_motor_output(const struct shell *sh,
						 const struct zros_topic *topic, const void *msg,
						 size_t msg_size)
{
	synapse_topic_MotorValues4f_t motors = {0};
	synapse_topic_MotorRaw4u16_t raw = {0};
	bool armed = false;
	bool test_mode = false;

	ARG_UNUSED(topic);

	if (!rdd2_topic_fb_unpack_motor_output(msg, msg_size, &motors, &raw, &armed, &test_mode)) {
		shell_error(sh, "motor_output: invalid snapshot");
		return;
	}

	shell_print(sh,
		    "motor armed=%1d test_mode=%1d out=[%6.3f/%4u %6.3f/%4u %6.3f/%4u %6.3f/%4u]",
		    armed ? 1 : 0, test_mode ? 1 : 0, (double)motors.m0, (unsigned int)raw.m0,
		    (double)motors.m1, (unsigned int)raw.m1, (double)motors.m2,
		    (unsigned int)raw.m2, (double)motors.m3, (unsigned int)raw.m3);
}

static void rdd2_topic_shell_format_rc(const struct shell *sh, const struct zros_topic *topic,
				       const void *msg, size_t msg_size)
{
	const synapse_topic_RcChannels16_t *rc = msg;

	ARG_UNUSED(topic);

	if (msg_size != sizeof(*rc)) {
		shell_error(sh, "rc: invalid sample size %u", (unsigned int)msg_size);
		return;
	}

	shell_print(sh, "rc ch0-7=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]", (long)rc->ch0,
		    (long)rc->ch1, (long)rc->ch2, (long)rc->ch3, (long)rc->ch4, (long)rc->ch5,
		    (long)rc->ch6, (long)rc->ch7);
	shell_print(sh, "rc ch8-15=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]", (long)rc->ch8,
		    (long)rc->ch9, (long)rc->ch10, (long)rc->ch11, (long)rc->ch12, (long)rc->ch13,
		    (long)rc->ch14, (long)rc->ch15);
}

static struct zros_shell_topic_formatter g_rdd2_flight_state_formatter = {
	.topic = &topic_flight_state,
	.format = rdd2_topic_shell_format_flight_state,
};

static struct zros_shell_topic_formatter g_rdd2_motor_output_formatter = {
	.topic = &topic_motor_output,
	.format = rdd2_topic_shell_format_motor_output,
};

static struct zros_shell_topic_formatter g_rdd2_rc_formatter = {
	.topic = &topic_rc,
	.format = rdd2_topic_shell_format_rc,
};

static int rdd2_topic_shell_init(void)
{
	int rc;

	rc = zros_shell_topic_formatter_register(&g_rdd2_flight_state_formatter);
	if (rc != 0 && rc != -EALREADY) {
		return rc;
	}

	rc = zros_shell_topic_formatter_register(&g_rdd2_motor_output_formatter);
	if (rc != 0 && rc != -EALREADY) {
		return rc;
	}

	rc = zros_shell_topic_formatter_register(&g_rdd2_rc_formatter);
	if (rc != 0 && rc != -EALREADY) {
		return rc;
	}

	return 0;
}

SYS_INIT(rdd2_topic_shell_init, APPLICATION, 0);
