/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_control.h"
#include "attitude_estimator.h"
#include "control_io.h"
#include "flight_mode.h"
#include "hotpath_memory.h"
#include "motor_output.h"
#include "rate_control.h"
#include "topic_shell.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rdd2, LOG_LEVEL_INF);

struct control_context {
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;
	synapse_topic_MotorValues4f_t motors;
	synapse_topic_MotorRaw4u16_t raw_test;
	float throttle_input;
	float throttle_cmd;
	float dt;
	int64_t now_ms;
	bool rc_stale;
	enum rdd2_flight_mode flight_mode;
};

/* Keep the persistent 800 Hz control-loop working set in DTCM on Tropic. */
static RDD2_HOTPATH_DTCM_BSS struct rdd2_attitude_controller g_attitude_controller;
static RDD2_HOTPATH_DTCM_BSS struct rdd2_attitude_estimator g_attitude_estimator;
static RDD2_HOTPATH_DTCM_BSS struct rdd2_rate_controller g_rate_controller;
static RDD2_HOTPATH_DTCM_BSS struct control_context g_control_ctx;

static void publish_flight_state(const struct control_context *ctx)
{
	rdd2_topic_flight_state_publish(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status,
					   &ctx->attitude, &ctx->attitude_desired,
					   &ctx->rate_desired, &ctx->rate_cmd);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	enum rdd2_flight_mode previous_mode = RDD2_FLIGHT_MODE_ACRO;
	bool was_armed = false;
	int rc;

	*ctx = (struct control_context){0};
	rdd2_attitude_controller_init(&g_attitude_controller);
	rdd2_attitude_estimator_init(&g_attitude_estimator);
	rdd2_rate_controller_init(&g_rate_controller);
	rdd2_motor_output_init();

	rc = rdd2_control_io_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("RDD2 flight stack starting");

	while (true) {
		was_armed = ctx->status.armed;
		previous_mode = ctx->flight_mode;

		rdd2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status, &ctx->dt);
		ctx->now_ms = k_uptime_get();
		ctx->rc_stale =
			!ctx->status.rc_valid ||
			((ctx->now_ms - ctx->status.rc_stamp_ms) > RDD2_RC_STALE_TIMEOUT_MS);
		ctx->flight_mode = rdd2_flight_mode_from_rc(&ctx->rc);
		ctx->status.flight_mode = (uint8_t)ctx->flight_mode;

		if (ctx->dt <= 0.0f) {
			ctx->dt = (float)RDD2_CONTROL_PERIOD_US * 1.0e-6f;
		}

		ctx->status.arm_switch = rdd2_rate_arm_switch_high(&ctx->rc);
		ctx->status.throttle_us = rdd2_rate_throttle_us(&ctx->rc);
		ctx->status.rc_stale = ctx->rc_stale;
		ctx->attitude = (synapse_topic_AttitudeEuler_t){0};
		ctx->attitude_desired = (synapse_topic_AttitudeEuler_t){0};
		ctx->rate_desired = (synapse_topic_RateTriplet_t){0};
		ctx->rate_cmd = (synapse_topic_RateTriplet_t){0};
		ctx->motors = (synapse_topic_MotorValues4f_t){0};

		if (!ctx->status.imu_ok || ctx->rc_stale || !ctx->status.arm_switch) {
			ctx->status.armed = false;
		} else if (!ctx->status.armed &&
			   ctx->status.throttle_us <= RDD2_THROTTLE_ARM_MAX) {
			ctx->status.armed = true;
		}

		if (!ctx->status.armed || !was_armed) {
			rdd2_attitude_estimator_reset_from_accel(&g_attitude_estimator,
							 &ctx->accel);
		} else if (ctx->status.imu_ok) {
			rdd2_attitude_estimator_predict(&g_attitude_estimator, &ctx->gyro,
							   &ctx->accel, ctx->dt);
		}

		if (ctx->status.imu_ok) {
			rdd2_attitude_estimator_get_attitude(&g_attitude_estimator,
							 &ctx->attitude);
			ctx->attitude_desired = ctx->attitude;
		}

		if (!ctx->status.armed || !was_armed || ctx->flight_mode != previous_mode) {
			rdd2_attitude_controller_reset(&g_attitude_controller);
			rdd2_rate_controller_reset(&g_rate_controller);
		}

		if (rdd2_motor_test_get(&ctx->motors)) {
			publish_flight_state(ctx);
			rdd2_motor_output_write_all(&ctx->motors, true, true);
			continue;
		}

		if (rdd2_motor_raw_test_get(&ctx->raw_test)) {
			publish_flight_state(ctx);
			rdd2_motor_output_write_all_raw(&ctx->raw_test, true);
			continue;
		}

		if (!ctx->status.imu_ok || ctx->rc_stale) {
			publish_flight_state(ctx);
			rdd2_motor_output_write_all(&ctx->motors, false, false);
			continue;
		}

		switch (ctx->flight_mode) {
		case RDD2_FLIGHT_MODE_AUTO_LEVEL:
			rdd2_attitude_desired_from_rc(&ctx->rc, &ctx->attitude,
							 &ctx->attitude_desired);
			rdd2_attitude_controller_step(&g_attitude_controller, &ctx->attitude,
							 &ctx->attitude_desired, &ctx->rc,
							 ctx->dt, &ctx->rate_desired);
			break;
		case RDD2_FLIGHT_MODE_ACRO:
		default:
			rdd2_rate_desired_from_rc(&ctx->rc, &ctx->rate_desired);
			ctx->attitude_desired = ctx->attitude;
			break;
		}

		ctx->throttle_input = rdd2_rate_throttle_input_from_rc(&ctx->rc);
		ctx->throttle_cmd =
			rdd2_rate_throttle_command(ctx->throttle_input, ctx->status.armed);
		rdd2_rate_controller_step(&g_rate_controller, &ctx->rate_desired,
					     &ctx->gyro, ctx->dt,
					     ctx->status.armed &&
						     ctx->throttle_input >
							     RDD2_PID_INTEGRATE_THROTTLE_MIN,
					     &ctx->rate_cmd);
		rdd2_mix_quad_x(ctx->throttle_cmd, &ctx->rate_cmd, &ctx->motors);
		publish_flight_state(ctx);
		rdd2_motor_output_write_all(&ctx->motors, ctx->status.armed, false);
	}

	return 0;
}
