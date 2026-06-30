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

/* Keep the persistent control-loop working set in DTCM if available. */
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
		pitch = -pitch;
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
	rc->ch0 = pwm_from_centered_stick((float)m->aileron, true);
	rc->ch1 = pwm_from_centered_stick((float)m->elevator, true);
	rc->ch2 = pwm_from_throttle((float)m->throttle);
	rc->ch3 = pwm_from_centered_stick((float)m->rudder, false);
	rc->ch4 = (int32_t)clampf_local((float)m->stabilizer, 1000.0f, 2000.0f);
	rc->ch5 = (int32_t)m->current_wp;
	rc->ch6 = (int32_t)(1000.0f * (float)m->des_v);
	rc->ch7 = (int32_t)(1000.0f * (float)m->phi_cmd);
	rc->ch8 = (int32_t)(1000.0f * (float)m->chi_err);
}

static bool fixed_wing_bridge_select_ppm_output(
	const synapse_topic_RcChannels16_t *manual_rc, const synapse_topic_RcChannels16_t *auto_rc,
	const synapse_topic_ControlStatus_t *status, synapse_topic_RcChannels16_t *out_rc)
{
#if defined(CONFIG_CUBS2_PPM_MANUAL_OVERRIDE)
	const int32_t *manual_channels = cubs2_topic_rc_channels_data_const(manual_rc);
#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	static int s_last_auto_mode = -1;
#endif
	bool manual_valid = status->rc_valid;
	int32_t switch_us = manual_channels[CONFIG_CUBS2_PPM_AUTO_SWITCH_CHANNEL];
	bool auto_mode;

	for (size_t i = 0U; i < 5U; i++) {
		manual_valid = manual_valid && (manual_channels[i] >= 900) &&
			       (manual_channels[i] <= 2100);
	}
	manual_valid = manual_valid && (switch_us >= 900) && (switch_us <= 2100);

	auto_mode = !manual_valid || (switch_us > CONFIG_CUBS2_PPM_AUTO_SWITCH_THRESHOLD_US);
	*out_rc = auto_mode ? *auto_rc : *manual_rc;

#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	if ((int)auto_mode != s_last_auto_mode) {
		if (manual_valid) {
			LOG_INF("PPM bridge mode: %s switch_ch%u=%ld",
				auto_mode ? "auto" : "manual",
				(unsigned int)CONFIG_CUBS2_PPM_AUTO_SWITCH_CHANNEL,
				(long)switch_us);
		} else {
			LOG_INF("PPM bridge mode: auto (manual RC unavailable)");
		}
		s_last_auto_mode = (int)auto_mode;
	}
#endif

	return auto_mode;
#else
	ARG_UNUSED(manual_rc);
	ARG_UNUSED(status);
	*out_rc = *auto_rc;
	return true;
#endif
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	const k_timeout_t controller_timeout =
		K_NSEC(CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_NS);
	int rc;

	*ctx = (struct control_context){0};
	CubControl_FixedWingOuterLoop_init(&g_model);
	g_model.dt = CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_S;

	rc = cubs2_control_io_init();
	if (rc != 0) {
		return rc;
	}

	rc = cubs2_serial_bridge_init();
	if (rc != 0) {
#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
		LOG_ERR("Failed to initialize serial bridge: %d", rc);
#endif
	}

	LOG_INF("CUBS2 Fixed-Wing Bridge starting");

	while (true) {
		// Wait for next telemetry packet or generated-period trigger. In SITL this
		// also stages the simulated mocap pose; on flight hardware it
		// leaves ctx->mocap invalid.
		cubs2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status,
					 CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_S,
					 controller_timeout, &ctx->dt, &ctx->mocap);
		ctx->now_ms = k_uptime_get();
		synapse_topic_RcChannels16_t manual_rc = ctx->rc;
		synapse_topic_RcChannels16_t auto_rc = {0};

		// Fall back to the serial bridge mocap source when no other source
		// has provided a valid pose this cycle.
		if (!ctx->mocap.valid) {
			cubs2_serial_bridge_get_mocap(&ctx->mocap);
		}

		// Update model inputs
		fixed_wing_bridge_map_input(&g_model, ctx);

		// Advance one generated-period discrete control step.
		CubControl_FixedWingOuterLoop_step(&g_model);

		// Map model outputs to autonomous RC sticks, then mirror the legacy
		// ROS ppm_bridge manual/autonomous switch behavior.
		fixed_wing_bridge_map_output(&g_model, &auto_rc);
		(void)fixed_wing_bridge_select_ppm_output(&manual_rc, &auto_rc, &ctx->status,
							  &ctx->rc);

#if defined(CONFIG_CUBS2_FWDBG_LOG)
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
#endif

		// Publish stick overrides to the bridge output
		publish_bridge_state(ctx);
		cubs2_topic_rc_published();

		// Send RC overrides back to Ubuntu bridge
		cubs2_serial_bridge_send_rc(&ctx->rc);
	}

	return 0;
}
