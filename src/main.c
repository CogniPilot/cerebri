/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"
#include "motor_output.h"
#include "rate_control.h"
#include "topic_shell.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cerebri, LOG_LEVEL_INF);

struct control_context {
	cerebri_topic_Vec3f_t gyro;
	cerebri_topic_Vec3f_t accel;
	cerebri_topic_RcChannels16_t rc;
	cerebri_topic_ControlStatus_t status;
	cerebri_topic_RateTriplet_t rate_desired;
	cerebri_topic_RateTriplet_t rate_cmd;
	cerebri_topic_MotorValues4f_t motors;
	cerebri_topic_MotorRaw4u16_t raw_test;
	float throttle_input;
	float throttle_cmd;
	float dt;
	int64_t now_ms;
	bool rc_stale;
};

static void publish_flight_state(const struct control_context *ctx)
{
	cerebri_topic_flight_state_publish(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status,
					   &ctx->rate_desired, &ctx->rate_cmd);
}

int main(void)
{
	struct cerebri_rate_controller rate_controller;
	struct control_context ctx = {0};
	int rc;

	cerebri_rate_controller_init(&rate_controller);
	cerebri_motor_output_init();

	rc = cerebri_control_io_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("cerebri rate-mode stack starting");

	while (true) {
		cerebri_control_input_wait(&ctx.gyro, &ctx.accel, &ctx.rc, &ctx.status, &ctx.dt);
		ctx.now_ms = k_uptime_get();
		ctx.rc_stale =
			!ctx.status.rc_valid ||
			((ctx.now_ms - ctx.status.rc_stamp_ms) > CEREBRI_RC_STALE_TIMEOUT_MS);

		if (ctx.dt <= 0.0f) {
			ctx.dt = (float)CEREBRI_CONTROL_PERIOD_US * 1.0e-6f;
		}

		if (cerebri_motor_test_get(&ctx.motors)) {
			cerebri_motor_output_write_all(&ctx.motors, true, true);
			continue;
		}

		if (cerebri_motor_raw_test_get(&ctx.raw_test)) {
			cerebri_motor_output_write_all_raw(&ctx.raw_test, true);
			continue;
		}

		ctx.status.arm_switch = cerebri_rate_arm_switch_high(&ctx.rc);
		ctx.status.throttle_us = cerebri_rate_throttle_us(&ctx.rc);

		if (!ctx.status.imu_ok || ctx.rc_stale || !ctx.status.arm_switch) {
			ctx.status.armed = false;
		} else if (!ctx.status.armed &&
			   ctx.status.throttle_us <= CEREBRI_THROTTLE_ARM_MAX) {
			ctx.status.armed = true;
			cerebri_rate_controller_reset(&rate_controller);
		}

		ctx.status.rc_stale = ctx.rc_stale;
		ctx.rate_desired = (cerebri_topic_RateTriplet_t){0};
		ctx.rate_cmd = (cerebri_topic_RateTriplet_t){0};

		if (!ctx.status.imu_ok || ctx.rc_stale) {
			publish_flight_state(&ctx);
			cerebri_motor_output_write_all(&ctx.motors, false, false);
			continue;
		}

		cerebri_rate_desired_from_rc(&ctx.rc, &ctx.rate_desired);
		ctx.throttle_input = cerebri_rate_throttle_input_from_rc(&ctx.rc);
		ctx.throttle_cmd =
			cerebri_rate_throttle_command(ctx.throttle_input, ctx.status.armed);
		cerebri_rate_controller_step(&rate_controller, &ctx.rate_desired, &ctx.gyro, ctx.dt,
					     ctx.status.armed &&
						     ctx.throttle_input >
							     CEREBRI_PID_INTEGRATE_THROTTLE_MIN,
					     &ctx.rate_cmd);
		cerebri_mix_quad_x(ctx.throttle_cmd, &ctx.rate_cmd, &ctx.motors);
		publish_flight_state(&ctx);
		cerebri_motor_output_write_all(&ctx.motors, ctx.status.armed, false);
	}

	return 0;
}
