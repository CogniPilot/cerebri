/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_shell.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static void fc_latest_state(cerebri_topic_Vec3f_t *gyro, cerebri_topic_Vec3f_t *accel,
			    cerebri_topic_RcChannels16_t *rc,
			    cerebri_topic_ControlStatus_t *status,
			    cerebri_topic_RateTriplet_t *rate_desired,
			    cerebri_topic_RateTriplet_t *rate_cmd)
{
	(void)cerebri_topic_flight_state_get(gyro, accel, rc, status, rate_desired, rate_cmd);
}

static int cmd_fc_status(const struct shell *sh, size_t argc, char **argv)
{
	cerebri_topic_Vec3f_t gyro = {0};
	cerebri_topic_Vec3f_t accel = {0};
	cerebri_topic_RcChannels16_t rc = {0};
	cerebri_topic_ControlStatus_t status = {0};
	cerebri_topic_RateTriplet_t rate_desired = {0};
	cerebri_topic_RateTriplet_t rate_cmd = {0};
	int64_t now_ms = k_uptime_get();
	int64_t rc_age_ms;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &rate_desired, &rate_cmd);
	rc_age_ms = status.rc_valid ? (now_ms - status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "armed=%d imu_ok=%d rc_valid=%d rc_stale=%d arm_switch=%d rc_age_ms=%lld lq=%u throttle_us=%ld",
		    status.armed ? 1 : 0,
		    status.imu_ok ? 1 : 0,
		    status.rc_valid ? 1 : 0,
		    status.rc_stale ? 1 : 0,
		    status.arm_switch ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)status.rc_link_quality,
		    (long)status.throttle_us);
	shell_print(sh,
		    "rate_desired roll=%0.3f pitch=%0.3f yaw=%0.3f rate_cmd roll=%0.3f pitch=%0.3f yaw=%0.3f",
		    (double)rate_desired.roll,
		    (double)rate_desired.pitch,
		    (double)rate_desired.yaw,
		    (double)rate_cmd.roll,
		    (double)rate_cmd.pitch,
		    (double)rate_cmd.yaw);

	return 0;
}

static int cmd_fc_gyro(const struct shell *sh, size_t argc, char **argv)
{
	cerebri_topic_Vec3f_t gyro = {0};
	cerebri_topic_Vec3f_t accel = {0};
	cerebri_topic_RcChannels16_t rc = {0};
	cerebri_topic_ControlStatus_t status = {0};
	cerebri_topic_RateTriplet_t rate_desired = {0};
	cerebri_topic_RateTriplet_t rate_cmd = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &rate_desired, &rate_cmd);

	shell_print(sh,
		    "gyro_rad_s x=%0.5f y=%0.5f z=%0.5f",
		    (double)gyro.x,
		    (double)gyro.y,
		    (double)gyro.z);
	shell_print(sh,
		    "accel_m_s2 x=%0.5f y=%0.5f z=%0.5f",
		    (double)accel.x,
		    (double)accel.y,
		    (double)accel.z);

	return 0;
}

static int cmd_fc_rc(const struct shell *sh, size_t argc, char **argv)
{
	cerebri_topic_Vec3f_t gyro = {0};
	cerebri_topic_Vec3f_t accel = {0};
	cerebri_topic_RcChannels16_t rc = {0};
	cerebri_topic_ControlStatus_t status = {0};
	cerebri_topic_RateTriplet_t rate_desired = {0};
	cerebri_topic_RateTriplet_t rate_cmd = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	fc_latest_state(&gyro, &accel, &rc, &status, &rate_desired, &rate_cmd);

	shell_print(sh,
		    "rc us ch0=%ld ch1=%ld ch2=%ld ch3=%ld ch4=%ld ch5=%ld ch6=%ld ch7=%ld",
		    (long)rc.ch0,
		    (long)rc.ch1,
		    (long)rc.ch2,
		    (long)rc.ch3,
		    (long)rc.ch4,
		    (long)rc.ch5,
		    (long)rc.ch6,
		    (long)rc.ch7);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_fc,
	SHELL_CMD(status, NULL, "show latest flight-control state", cmd_fc_status),
	SHELL_CMD(gyro, NULL, "show latest gyro and accel sample", cmd_fc_gyro),
	SHELL_CMD(rc, NULL, "show latest RC channel sample", cmd_fc_rc),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(fc, &sub_fc, "flight-control debug commands", NULL);
