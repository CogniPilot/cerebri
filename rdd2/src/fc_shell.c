/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flight_mode.h"
#include "imu_latency_stats.h"
#include "topic_bus.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zros/zros_topic.h>

static void fc_print_latency_stats(const struct shell *sh)
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

static void fc_latest_rc(synapse_topic_RcChannels16_t *rc)
{
	memset(rc, 0, sizeof(*rc));

	if (!rdd2_topic_has_sample(&topic_rc)) {
		return;
	}

	(void)zros_topic_read(&topic_rc, rc);
}

static void fc_latest_state(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel,
			    synapse_topic_RcChannels16_t *rc, synapse_topic_ControlStatus_t *status,
			    synapse_topic_AttitudeEuler_t *attitude,
			    synapse_topic_AttitudeEuler_t *attitude_desired,
			    synapse_topic_RateTriplet_t *rate_desired,
			    synapse_topic_RateTriplet_t *rate_cmd, uint32_t *main_loop_latency_us)
{
	rdd2_topic_flight_state_blob_t blob = {0};

	memset(gyro, 0, sizeof(*gyro));
	memset(accel, 0, sizeof(*accel));
	memset(rc, 0, sizeof(*rc));
	memset(status, 0, sizeof(*status));
	memset(attitude, 0, sizeof(*attitude));
	memset(attitude_desired, 0, sizeof(*attitude_desired));
	memset(rate_desired, 0, sizeof(*rate_desired));
	memset(rate_cmd, 0, sizeof(*rate_cmd));
	if (main_loop_latency_us != NULL) {
		*main_loop_latency_us = 0U;
	}

	if (!rdd2_topic_has_sample(&topic_flight_state)) {
		return;
	}

	if (zros_topic_read(&topic_flight_state, &blob) != 0) {
		return;
	}

	(void)rdd2_topic_fb_unpack_flight_state(blob, sizeof(blob), gyro, accel, rc, status,
						attitude, attitude_desired, rate_desired, rate_cmd,
						main_loop_latency_us);
}

static int cmd_fc_status(const struct shell *sh, size_t argc, char **argv)
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

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired, &rate_desired,
			&rate_cmd, &main_loop_latency_us);
	rc_age_ms = status.rc_valid ? (now_ms - status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "mode=%s armed=%d imu_ok=%d rc_valid=%d rc_stale=%d arm_switch=%d "
		    "rc_age_ms=%lld lq=%u throttle_us=%ld imu_to_motor_us=%lu",
		    rdd2_flight_mode_name((enum rdd2_flight_mode)status.flight_mode),
		    status.armed ? 1 : 0, status.imu_ok ? 1 : 0, status.rc_valid ? 1 : 0,
		    status.rc_stale ? 1 : 0, status.arm_switch ? 1 : 0, (long long)rc_age_ms,
		    (unsigned int)status.rc_link_quality, (long)status.throttle_us,
		    (unsigned long)main_loop_latency_us);
	fc_print_latency_stats(sh);
	shell_print(sh,
		    "rate_desired roll=%0.3f pitch=%0.3f yaw=%0.3f rate_cmd roll=%0.3f pitch=%0.3f "
		    "yaw=%0.3f",
		    (double)rate_desired.roll, (double)rate_desired.pitch, (double)rate_desired.yaw,
		    (double)rate_cmd.roll, (double)rate_cmd.pitch, (double)rate_cmd.yaw);
	shell_print(sh,
		    "attitude roll=%0.3f pitch=%0.3f yaw=%0.3f attitude_desired roll=%0.3f "
		    "pitch=%0.3f yaw=%0.3f",
		    (double)attitude.roll, (double)attitude.pitch, (double)attitude.yaw,
		    (double)attitude_desired.roll, (double)attitude_desired.pitch,
		    (double)attitude_desired.yaw);

	return 0;
}

static int cmd_fc_gyro(const struct shell *sh, size_t argc, char **argv)
{
	synapse_topic_Vec3f_t gyro = {0};
	synapse_topic_Vec3f_t accel = {0};
	synapse_topic_RcChannels16_t rc = {0};
	synapse_topic_ControlStatus_t status = {0};
	synapse_topic_AttitudeEuler_t attitude = {0};
	synapse_topic_AttitudeEuler_t attitude_desired = {0};
	synapse_topic_RateTriplet_t rate_desired = {0};
	synapse_topic_RateTriplet_t rate_cmd = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired, &rate_desired,
			&rate_cmd, NULL);

	shell_print(sh, "gyro_rad_s x=%0.5f y=%0.5f z=%0.5f", (double)gyro.x, (double)gyro.y,
		    (double)gyro.z);
	shell_print(sh, "accel_m_s2 x=%0.5f y=%0.5f z=%0.5f", (double)accel.x, (double)accel.y,
		    (double)accel.z);

	return 0;
}

static int cmd_fc_rc(const struct shell *sh, size_t argc, char **argv)
{
	synapse_topic_Vec3f_t gyro = {0};
	synapse_topic_Vec3f_t accel = {0};
	synapse_topic_RcChannels16_t rc = {0};
	synapse_topic_ControlStatus_t status = {0};
	synapse_topic_AttitudeEuler_t attitude = {0};
	synapse_topic_AttitudeEuler_t attitude_desired = {0};
	synapse_topic_RateTriplet_t rate_desired = {0};
	synapse_topic_RateTriplet_t rate_cmd = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired, &rate_desired,
			&rate_cmd, NULL);
	fc_latest_rc(&rc);

	shell_print(sh, "rc us ch0=%ld ch1=%ld ch2=%ld ch3=%ld ch4=%ld ch5=%ld ch6=%ld ch7=%ld",
		    (long)rc.ch0, (long)rc.ch1, (long)rc.ch2, (long)rc.ch3, (long)rc.ch4,
		    (long)rc.ch5, (long)rc.ch6, (long)rc.ch7);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_fc, SHELL_CMD(status, NULL, "show latest flight-control state", cmd_fc_status),
	SHELL_CMD(gyro, NULL, "show latest gyro and accel sample", cmd_fc_gyro),
	SHELL_CMD(rc, NULL, "show latest RC channel sample", cmd_fc_rc), SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(fc, &sub_fc, "flight-control debug commands", NULL);
