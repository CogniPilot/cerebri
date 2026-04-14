/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_control.h"
#include "attitude_estimator.h"
#include "control_io.h"
#include "flight_mode.h"
#include "hotpath_memory.h"
#include "imu_latency_stats.h"
#include "motor_output.h"
#include "rate_control.h"
#include "topic_bus.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/zros_node.h>
#include <zros/zros_pub.h>

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
	uint32_t imu_to_motor_latency_us;
	int64_t now_ms;
	bool rc_stale;
	enum rdd2_flight_mode flight_mode;
};

/* Keep the persistent 1600 Hz rate-loop working set in DTCM on Tropic. */
static RDD2_HOTPATH_DTCM_BSS struct rdd2_attitude_controller g_attitude_controller;
static RDD2_HOTPATH_DTCM_BSS struct rdd2_attitude_estimator g_attitude_estimator;
static RDD2_HOTPATH_DTCM_BSS struct rdd2_rate_controller g_rate_controller;
static RDD2_HOTPATH_DTCM_BSS struct control_context g_control_ctx;
static struct zros_node g_rdd2_main_node;
static struct zros_pub g_rdd2_flight_state_pub;
static RDD2_HOTPATH_DTCM_BSS rdd2_topic_flight_state_blob_t g_rdd2_flight_state_blob;

static bool loop_divider_expired(uint32_t *countdown, uint32_t divisor)
{
	if (*countdown > 0U) {
		(*countdown)--;
		return false;
	}

	*countdown = divisor - 1U;
	return true;
}

static bool attitude_update_due(enum rdd2_flight_mode mode, bool armed,
				uint32_t *autolevel_countdown, uint32_t *background_countdown)
{
	if (armed && mode == RDD2_FLIGHT_MODE_AUTO_LEVEL) {
		return loop_divider_expired(autolevel_countdown, RDD2_ATTITUDE_AUTOLEVEL_DIV);
	}

	return loop_divider_expired(background_countdown, RDD2_ATTITUDE_BACKGROUND_DIV);
}

static int flight_state_topic_init(void)
{
	zros_node_init(&g_rdd2_main_node, "rdd2_main");
	return zros_pub_init(&g_rdd2_flight_state_pub, &g_rdd2_main_node, &topic_flight_state,
			     &g_rdd2_flight_state_blob);
}

static void publish_flight_state(const struct control_context *ctx)
{
	static uint32_t publish_countdown;
	size_t len;

	if (!loop_divider_expired(&publish_countdown, RDD2_FLIGHT_STATE_PUBLISH_DIV)) {
		return;
	}

	len = rdd2_topic_fb_pack_flight_state(
		g_rdd2_flight_state_blob, sizeof(g_rdd2_flight_state_blob), &ctx->gyro, &ctx->accel,
		&ctx->rc, &ctx->status, &ctx->attitude, &ctx->attitude_desired, &ctx->rate_desired,
		&ctx->rate_cmd, ctx->imu_to_motor_latency_us);
	if (len != sizeof(g_rdd2_flight_state_blob)) {
		return;
	}

	(void)zros_pub_update(&g_rdd2_flight_state_pub);
}

static uint32_t imu_to_motor_latency_us(uint64_t imu_interrupt_timestamp_ns,
					uint64_t motor_signal_timestamp_ns)
{
	uint64_t latency_ns;

	if (imu_interrupt_timestamp_ns == 0U || motor_signal_timestamp_ns == 0U ||
	    motor_signal_timestamp_ns <= imu_interrupt_timestamp_ns) {
		return 0U;
	}

	latency_ns = motor_signal_timestamp_ns - imu_interrupt_timestamp_ns;
	if (latency_ns >= ((uint64_t)UINT32_MAX * 1000U)) {
		return UINT32_MAX;
	}

	return (uint32_t)(latency_ns / 1000U);
}

static void finalize_cycle(struct control_context *ctx, uint64_t imu_interrupt_timestamp_ns,
			   uint64_t motor_signal_timestamp_ns)
{
	ctx->imu_to_motor_latency_us =
		imu_to_motor_latency_us(imu_interrupt_timestamp_ns, motor_signal_timestamp_ns);
	rdd2_imu_latency_stats_update(ctx->imu_to_motor_latency_us);
	publish_flight_state(ctx);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	enum rdd2_flight_mode previous_mode = RDD2_FLIGHT_MODE_ACRO;
	bool was_armed = false;
	uint32_t autolevel_attitude_countdown = 0U;
	uint32_t background_attitude_countdown = 0U;
	synapse_topic_Vec3f_t attitude_gyro_accum = {0};
	synapse_topic_Vec3f_t attitude_accel_accum = {0};
	float attitude_dt_accum = 0.0f;
	uint32_t attitude_sample_count = 0U;
	int rc;

	*ctx = (struct control_context){0};
	rdd2_attitude_controller_init(&g_attitude_controller);
	rdd2_attitude_estimator_init(&g_attitude_estimator);
	rdd2_rate_controller_init(&g_rate_controller);
	rdd2_imu_latency_stats_reset();

	rc = flight_state_topic_init();
	if (rc != 0) {
		return rc;
	}

	rdd2_motor_output_init();

	rc = rdd2_control_io_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("RDD2 flight stack starting");

	while (true) {
		bool run_attitude_update;
		bool controller_reset_required;
		float attitude_dt = 0.0f;
		uint64_t imu_interrupt_timestamp_ns = 0U;
		uint64_t motor_signal_timestamp_ns = 0U;

		was_armed = ctx->status.armed;
		previous_mode = ctx->flight_mode;

		rdd2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status, &ctx->dt,
					&imu_interrupt_timestamp_ns);
		ctx->now_ms = k_uptime_get();
		ctx->rc_stale = !ctx->status.rc_valid || ((ctx->now_ms - ctx->status.rc_stamp_ms) >
							  RDD2_RC_STALE_TIMEOUT_MS);
		ctx->flight_mode = rdd2_flight_mode_from_rc(&ctx->rc);
		ctx->status.flight_mode = (uint8_t)ctx->flight_mode;

		if (ctx->dt <= 0.0f) {
			ctx->dt = RDD2_CONTROL_DT_S;
		}

		ctx->status.arm_switch = rdd2_rate_arm_switch_high(&ctx->rc);
		ctx->status.throttle_us = rdd2_rate_throttle_us(&ctx->rc);
		ctx->status.rc_stale = ctx->rc_stale;
		ctx->rate_cmd = (synapse_topic_RateTriplet_t){0};
		ctx->motors = (synapse_topic_MotorValues4f_t){0};
		ctx->imu_to_motor_latency_us = 0U;

		if (!ctx->status.imu_ok || ctx->rc_stale || !ctx->status.arm_switch) {
			ctx->status.armed = false;
		} else if (!ctx->status.armed && ctx->status.throttle_us <= RDD2_THROTTLE_ARM_MAX) {
			ctx->status.armed = true;
		}

		run_attitude_update = attitude_update_due(ctx->flight_mode, ctx->status.armed,
							  &autolevel_attitude_countdown,
							  &background_attitude_countdown);
		controller_reset_required =
			!ctx->status.armed || !was_armed || ctx->flight_mode != previous_mode;

		if (ctx->status.imu_ok) {
			attitude_gyro_accum.x += ctx->gyro.x;
			attitude_gyro_accum.y += ctx->gyro.y;
			attitude_gyro_accum.z += ctx->gyro.z;
			attitude_accel_accum.x += ctx->accel.x;
			attitude_accel_accum.y += ctx->accel.y;
			attitude_accel_accum.z += ctx->accel.z;
			attitude_dt_accum += ctx->dt;
			attitude_sample_count++;
		} else {
			attitude_gyro_accum = (synapse_topic_Vec3f_t){0};
			attitude_accel_accum = (synapse_topic_Vec3f_t){0};
			attitude_dt_accum = 0.0f;
			attitude_sample_count = 0U;
			ctx->attitude = (synapse_topic_AttitudeEuler_t){0};
			ctx->attitude_desired = (synapse_topic_AttitudeEuler_t){0};
			ctx->rate_desired = (synapse_topic_RateTriplet_t){0};
		}

		if (!ctx->status.armed) {
			if (ctx->status.imu_ok && run_attitude_update) {
				rdd2_attitude_estimator_reset_from_accel(&g_attitude_estimator,
									 &ctx->accel);
				rdd2_attitude_estimator_get_attitude(&g_attitude_estimator,
								     &ctx->attitude);
				ctx->attitude_desired = ctx->attitude;
				attitude_gyro_accum = (synapse_topic_Vec3f_t){0};
				attitude_accel_accum = (synapse_topic_Vec3f_t){0};
				attitude_dt_accum = 0.0f;
				attitude_sample_count = 0U;
			}
		} else if (ctx->status.imu_ok && run_attitude_update &&
			   attitude_sample_count > 0U) {
			float sample_scale = 1.0f / (float)attitude_sample_count;
			synapse_topic_Vec3f_t avg_gyro = {
				.x = attitude_gyro_accum.x * sample_scale,
				.y = attitude_gyro_accum.y * sample_scale,
				.z = attitude_gyro_accum.z * sample_scale,
			};
			synapse_topic_Vec3f_t avg_accel = {
				.x = attitude_accel_accum.x * sample_scale,
				.y = attitude_accel_accum.y * sample_scale,
				.z = attitude_accel_accum.z * sample_scale,
			};

			attitude_dt = attitude_dt_accum;
			rdd2_attitude_estimator_predict(&g_attitude_estimator, &avg_gyro,
							&avg_accel, attitude_dt);
			rdd2_attitude_estimator_get_attitude(&g_attitude_estimator, &ctx->attitude);
			attitude_gyro_accum = (synapse_topic_Vec3f_t){0};
			attitude_accel_accum = (synapse_topic_Vec3f_t){0};
			attitude_dt_accum = 0.0f;
			attitude_sample_count = 0U;
		}

		if (controller_reset_required) {
			rdd2_attitude_controller_reset(&g_attitude_controller);
			rdd2_rate_controller_reset(&g_rate_controller);
		}

		if (rdd2_motor_test_get(&ctx->motors)) {
			motor_signal_timestamp_ns =
				rdd2_motor_output_write_all(&ctx->motors, true, true);
			finalize_cycle(ctx, imu_interrupt_timestamp_ns, motor_signal_timestamp_ns);
			continue;
		}

		if (rdd2_motor_raw_test_get(&ctx->raw_test)) {
			motor_signal_timestamp_ns =
				rdd2_motor_output_write_all_raw(&ctx->raw_test, true);
			finalize_cycle(ctx, imu_interrupt_timestamp_ns, motor_signal_timestamp_ns);
			continue;
		}

		if (!ctx->status.imu_ok || ctx->rc_stale) {
			motor_signal_timestamp_ns =
				rdd2_motor_output_write_all(&ctx->motors, false, false);
			finalize_cycle(ctx, imu_interrupt_timestamp_ns, motor_signal_timestamp_ns);
			continue;
		}

		switch (ctx->flight_mode) {
		case RDD2_FLIGHT_MODE_AUTO_LEVEL:
			if (ctx->status.imu_ok && (controller_reset_required ||
						   (run_attitude_update && attitude_dt > 0.0f))) {
				rdd2_attitude_desired_from_rc(&ctx->rc, &ctx->attitude,
							      &ctx->attitude_desired);
				rdd2_attitude_controller_step(
					&g_attitude_controller, &ctx->attitude,
					&ctx->attitude_desired, &ctx->rc,
					(attitude_dt > 0.0f) ? attitude_dt : ctx->dt,
					&ctx->rate_desired);
			}
			ctx->rate_desired.yaw = rdd2_rate_yaw_desired_from_rc(&ctx->rc);
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
		rdd2_rate_controller_step(
			&g_rate_controller, &ctx->rate_desired, &ctx->gyro, ctx->dt,
			ctx->status.armed && ctx->throttle_input > RDD2_PID_INTEGRATE_THROTTLE_MIN,
			&ctx->rate_cmd);
		rdd2_mix_quad_x(ctx->throttle_cmd, &ctx->rate_cmd, &ctx->motors);
		motor_signal_timestamp_ns =
			rdd2_motor_output_write_all(&ctx->motors, ctx->status.armed, false);
		finalize_cycle(ctx, imu_interrupt_timestamp_ns, motor_signal_timestamp_ns);
	}

	return 0;
}
