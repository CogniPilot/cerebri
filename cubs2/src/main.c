/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"
#include "hotpath_memory.h"
#include "topic_shell.h"
#include "serial_bridge.h"
#include "generated_fixed_wing/CubControl_FixedWingOuterLoop.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cubs2, LOG_LEVEL_INF);

struct control_context {
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;
	cubs2_mocap_rigid_body_t mocap;
	float dt;
	int64_t now_ms;
};

/* Keep the persistent 100 Hz control-loop working set in DTCM if available. */
static CUBS2_HOTPATH_DTCM_BSS CubControl_FixedWingOuterLoop_t g_model;
static CUBS2_HOTPATH_DTCM_BSS struct control_context g_control_ctx;

static void publish_bridge_state(const struct control_context *ctx)
{
	cubs2_topic_flight_state_publish(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status,
					   &ctx->attitude, &ctx->attitude_desired,
					   &ctx->rate_desired, &ctx->rate_cmd);
}

static float clampf_local(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static int32_t pwm_from_centered_stick(float value, bool inverted)
{
	const float sign = inverted ? -1.0f : 1.0f;
	return (int32_t)(1500.0f + 500.0f * sign * clampf_local(value, -1.0f, 1.0f));
}

static int32_t pwm_from_throttle(float value)
{
	return (int32_t)(1000.0f + 1000.0f * clampf_local(value, 0.0f, 1.0f));
}

static void quat_to_euler(const cubs2_mocap_rigid_body_t *mocap, float *roll, float *pitch,
				  float *yaw)
{
	const float qw = mocap->qw;
	const float qx = mocap->qx;
	const float qy = mocap->qy;
	const float qz = mocap->qz;
	const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
	const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
	const float sinp = 2.0f * (qw * qy - qz * qx);
	const float siny_cosp = 2.0f * (qw * qz + qx * qy);
	const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

	*roll = atan2f(sinr_cosp, cosr_cosp);
	*pitch = asinf(clampf_local(sinp, -1.0f, 1.0f));
	*yaw = atan2f(siny_cosp, cosy_cosp);
}

static void fixed_wing_bridge_map_input(CubControl_FixedWingOuterLoop_t *m, const struct control_context *ctx)
{
	float roll = ctx->accel.x;
	float pitch = ctx->accel.y;
	float yaw = ctx->accel.z;

	if (ctx->mocap.valid) {
		quat_to_euler(&ctx->mocap, &roll, &pitch, &yaw);
		m->x = ctx->mocap.x;
		m->y = ctx->mocap.y;
		m->z = ctx->mocap.z;
	} else {
		m->x = ctx->gyro.x;
		m->y = ctx->gyro.y;
		m->z = ctx->gyro.z;
	}

	m->roll = roll;
	m->pitch = pitch;
	m->yaw = yaw;
}

static void fixed_wing_bridge_map_output(const CubControl_FixedWingOuterLoop_t *m, synapse_topic_RcChannels16_t *rc)
{
	rc->ch0 = pwm_from_centered_stick((float)m->aileron, false);
	rc->ch1 = pwm_from_centered_stick((float)m->elevator, true);
	rc->ch2 = pwm_from_throttle((float)m->throttle);
	rc->ch3 = pwm_from_centered_stick((float)m->rudder, false);
	rc->ch4 = (int32_t)clampf_local((float)m->stabilizer, 1000.0f, 2000.0f);
	rc->ch5 = (int32_t)m->current_wp;
	rc->ch6 = (int32_t)(1000.0f * (float)m->des_v);
	rc->ch7 = (int32_t)(1000.0f * (float)m->phi_cmd);
	rc->ch8 = (int32_t)(1000.0f * (float)m->chi_err);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	int rc;

	*ctx = (struct control_context){0};
	CubControl_FixedWingOuterLoop_init(&g_model);

	rc = cubs2_control_io_init();
	if (rc != 0) {
		return rc;
	}

	rc = cubs2_serial_bridge_init();
	if (rc != 0) {
		LOG_ERR("Failed to initialize serial bridge: %d", rc);
	}

	LOG_INF("CUBS2 Fixed-Wing Bridge starting");

	while (true) {
		// Wait for next telemetry packet or 100Hz trigger. In SITL this
		// also stages the simulated mocap pose; on flight hardware it
		// leaves ctx->mocap invalid.
		cubs2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status, &ctx->dt,
					 &ctx->mocap);
		ctx->now_ms = k_uptime_get();

		// Fall back to the serial bridge mocap source when no other source
		// has provided a valid pose this cycle.
		if (!ctx->mocap.valid) {
			cubs2_serial_bridge_get_mocap(&ctx->mocap);
		}

		// Update model inputs
		fixed_wing_bridge_map_input(&g_model, ctx);

		// Advance one 100 Hz discrete control step. The GALEC-generated model
		// has a fixed sample period baked into the controller state.
		CubControl_FixedWingOuterLoop_step(&g_model);

		// Map model outputs to RC sticks
		fixed_wing_bridge_map_output(&g_model, &ctx->rc);

		LOG_INF("FWDBG,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f",
			(double)g_model.time_s,
			(double)g_model.x, (double)g_model.y,
			(double)g_model.z, (double)g_model.roll,
			(double)g_model.pitch, (double)g_model.yaw,
			(double)g_model.aileron, (double)g_model.elevator,
			(double)g_model.throttle, (double)g_model.rudder,
			(double)g_model.stabilizer, (double)g_model.des_heading,
			(double)g_model.vx_est, (double)g_model.vy_est,
			(double)g_model.phi_cmd, (double)g_model.chi_err,
			(int)g_model.current_wp, (int)g_model.airborne,
			(double)g_model.des_v, (double)g_model.des_gamma);

		// Publish stick overrides to the bridge output
		publish_bridge_state(ctx);
		cubs2_topic_rc_published();

		// Send RC overrides back to Ubuntu bridge
		cubs2_serial_bridge_send_rc(&ctx->rc);
	}

	return 0;
}
