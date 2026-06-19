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

/* Generated-model p[] slot aliases (rumoca embedded-c). Gains, waypoints and
 * the rest of the parameters are baked into the model and set by
 * CubControl_FixedWingOuterLoop_init(); only the pose inputs and the control
 * outputs are exchanged each cycle. */
#define P_X          CUBCONTROL_FIXEDWINGOUTERLOOP_P_x
#define P_Y          CUBCONTROL_FIXEDWINGOUTERLOOP_P_y
#define P_Z          CUBCONTROL_FIXEDWINGOUTERLOOP_P_z
#define P_ROLL       CUBCONTROL_FIXEDWINGOUTERLOOP_P_roll
#define P_PITCH      CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch
#define P_YAW        CUBCONTROL_FIXEDWINGOUTERLOOP_P_yaw
#define P_AILERON    CUBCONTROL_FIXEDWINGOUTERLOOP_P_aileron
#define P_ELEVATOR   CUBCONTROL_FIXEDWINGOUTERLOOP_P_elevator
#define P_THROTTLE   CUBCONTROL_FIXEDWINGOUTERLOOP_P_throttle
#define P_RUDDER     CUBCONTROL_FIXEDWINGOUTERLOOP_P_rudder
#define P_STABILIZER CUBCONTROL_FIXEDWINGOUTERLOOP_P_stabilizer
#define P_DES_V      CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_v
#define P_DES_GAMMA  CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_gamma
#define P_DES_HEADING CUBCONTROL_FIXEDWINGOUTERLOOP_P_des_heading
#define P_CURRENT_WP CUBCONTROL_FIXEDWINGOUTERLOOP_P_current_wp
#define P_AIRBORNE   CUBCONTROL_FIXEDWINGOUTERLOOP_P_airborne
#define P_PHI_CMD    CUBCONTROL_FIXEDWINGOUTERLOOP_P_phi_cmd
#define P_CHI_ERR    CUBCONTROL_FIXEDWINGOUTERLOOP_P_chi_err
#define P_VX_EST     CUBCONTROL_FIXEDWINGOUTERLOOP_P_vx_est
#define P_VY_EST     CUBCONTROL_FIXEDWINGOUTERLOOP_P_vy_est

static void fixed_wing_bridge_map_input(CubControl_FixedWingOuterLoop_t *m, const struct control_context *ctx)
{
	float roll = ctx->accel.x;
	float pitch = ctx->accel.y;
	float yaw = ctx->accel.z;

	if (ctx->mocap.valid) {
		quat_to_euler(&ctx->mocap, &roll, &pitch, &yaw);
		m->p[P_X] = ctx->mocap.x;
		m->p[P_Y] = ctx->mocap.y;
		m->p[P_Z] = ctx->mocap.z;
	} else {
		m->p[P_X] = ctx->gyro.x;
		m->p[P_Y] = ctx->gyro.y;
		m->p[P_Z] = ctx->gyro.z;
	}

	m->p[P_ROLL] = roll;
	m->p[P_PITCH] = pitch;
	m->p[P_YAW] = yaw;
}

static void fixed_wing_bridge_map_output(const CubControl_FixedWingOuterLoop_t *m, synapse_topic_RcChannels16_t *rc)
{
	rc->ch0 = pwm_from_centered_stick((float)m->p[P_AILERON], false);
	rc->ch1 = pwm_from_centered_stick((float)m->p[P_ELEVATOR], true);
	rc->ch2 = pwm_from_throttle((float)m->p[P_THROTTLE]);
	rc->ch3 = pwm_from_centered_stick((float)m->p[P_RUDDER], false);
	rc->ch4 = (int32_t)clampf_local((float)m->p[P_STABILIZER], 1000.0f, 2000.0f);
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

		// Advance one 100 Hz discrete control step (snapshots pre(), runs the
		// sample tick). NOTE: step the model with its OWN dt (p[0], a double
		// 0.01) rather than ctx->dt, which is a float 0.01f. The generated
		// sample() event only fires when m->time is within 1e-9 of a multiple
		// of dt; accumulating m->time from the float (0.00999999977648...)
		// drifts ~2.2e-10 per step and silently stops the sample event after a
		// few steps, freezing the controller in open-loop takeoff.
		CubControl_FixedWingOuterLoop_step(&g_model, g_model.p[0]);

		// Map model outputs to RC sticks
		fixed_wing_bridge_map_output(&g_model, &ctx->rc);

		LOG_INF("FWDBG,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f",
			(double)g_model.time,
			(double)g_model.p[P_X], (double)g_model.p[P_Y],
			(double)g_model.p[P_Z], (double)g_model.p[P_ROLL],
			(double)g_model.p[P_PITCH], (double)g_model.p[P_YAW],
			(double)g_model.p[P_AILERON], (double)g_model.p[P_ELEVATOR],
			(double)g_model.p[P_THROTTLE], (double)g_model.p[P_RUDDER],
			(double)g_model.p[P_STABILIZER], (double)g_model.p[P_DES_HEADING],
			(double)g_model.p[P_VX_EST], (double)g_model.p[P_VY_EST],
			(double)g_model.p[P_PHI_CMD], (double)g_model.p[P_CHI_ERR],
			(int)g_model.p[P_CURRENT_WP], (int)g_model.p[P_AIRBORNE],
			(double)g_model.p[P_DES_V], (double)g_model.p[P_DES_GAMMA]);

		// Publish stick overrides to the bridge output
		publish_bridge_state(ctx);
		cubs2_topic_rc_published();

		// Send RC overrides back to Ubuntu bridge
		cubs2_serial_bridge_send_rc(&ctx->rc);
	}

	return 0;
}
