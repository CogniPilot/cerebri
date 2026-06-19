/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

static void init_parameters(CubControl_FixedWingOuterLoop_t *m)
{
	m->g = 9.81;
	m->nWaypoints = 6.0;
	m->waypoints[0][0] = -4.0f;   m->waypoints[0][1] = -5.0f;   m->waypoints[0][2] = 3.0f;
	m->waypoints[1][0] = -3.0f;   m->waypoints[1][1] = 2.0f;    m->waypoints[1][2] = 3.0f;
	m->waypoints[2][0] = 16.20f;  m->waypoints[2][1] = 2.0f;    m->waypoints[2][2] = 3.0f;
	m->waypoints[3][0] = 16.0f;   m->waypoints[3][1] = -4.22f;  m->waypoints[3][2] = 3.0f;
	m->waypoints[4][0] = 6.88f;   m->waypoints[4][1] = -5.1f;   m->waypoints[4][2] = 3.0f;
	m->waypoints[5][0] = -4.0f;   m->waypoints[5][1] = -5.0f;   m->waypoints[5][2] = 3.0f;
	m->filterCutoffHz = 10.0f;
	m->waypointSwitchingDistance = 4.0f;
	m->lookaheadTime = 1.5f;
	m->lookaheadMin = 1.0f;
	m->lookaheadMax = 5.0f;
	m->vCruise = 3.0f;
	m->takeoffAltitude = 0.4f;
	m->takeoffElev = 0.15f;
	m->mass = 0.057f;
	m->trimThrust = 3.5f;
	m->trimElev = 0.20f;
	m->trimAil = 0.0f;
	m->thrMax = 7.5f;
	m->envelopeDrag = 1.0f;
	m->stabilizerCmd = 2000.0f;
	m->K_thrustp = 0.01f;
	m->K_thrusti = 0.4215f;
	m->K_pitchp = 0.075f;
	m->K_pitchi = 0.216f;
	m->K_elevp = 0.107f;
	m->K_elevi = 0.2107f;
	m->K_q = 0.2f;
	m->K_phi_elev = 2.5f;
	m->K_deltap = 0.4f;
	m->K_deltai = 0.15f;
	m->K_deltad = 0.10f;
	m->pitchIntegralMax = 0.3f;
	m->normEsDotIntegralMax = 7.5f;
	m->distTermIntegralMax = 7.5f;
	m->rIntegralMax = 0.4f;
	m->kChi = 1.20f;
	m->phiLim = 30.0f * (float)M_PI / 180.0f;
	m->phiDotLim = 90.0f * (float)M_PI / 180.0f;
	m->chiDeadband = 1.0f * (float)M_PI / 180.0f;
	m->pitchCmdLim = 20.0f * (float)M_PI / 180.0f;
	m->dt = (float)CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_S;

	m->stabilizer = 1900.0f;
}

static void map_input(CubControl_FixedWingOuterLoop_t *m,
		      const struct control_context *ctx)
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

static void map_output(const CubControl_FixedWingOuterLoop_t *m,
		       synapse_topic_RcChannels16_t *rc)
{
	rc->ch0 = pwm_from_centered_stick((float)m->aileron, false);
	rc->ch1 = pwm_from_centered_stick((float)m->elevator, true);
	rc->ch2 = pwm_from_throttle((float)m->throttle);
	rc->ch3 = pwm_from_centered_stick((float)m->rudder, false);
	rc->ch4 = (int32_t)clampf_local((float)m->stabilizer, 1000.0f, 2000.0f);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	int rc;

	*ctx = (struct control_context){0};
	CubControl_FixedWingOuterLoop_init(&g_model);
	init_parameters(&g_model);

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
		cubs2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc,
					 &ctx->status, &ctx->dt);
		ctx->now_ms = k_uptime_get();

		cubs2_serial_bridge_get_mocap(&ctx->mocap);

		map_input(&g_model, ctx);

		CubControl_FixedWingOuterLoop_step(&g_model);

		map_output(&g_model, &ctx->rc);

		LOG_INF("FWDBG,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f",
			(double)g_model.time,
			(double)g_model.x, (double)g_model.y,
			(double)g_model.z, (double)g_model.roll,
			(double)g_model.pitch, (double)g_model.yaw,
			(double)g_model.aileron, (double)g_model.elevator,
			(double)g_model.throttle, (double)g_model.rudder,
			(double)g_model.stabilizer, (double)g_model.des_heading,
			(double)g_model.phi_cmd, (double)g_model.chi_err,
			(int)g_model.current_wp, (int)g_model.airborne,
			(double)g_model.des_v, (double)g_model.des_gamma);

		publish_bridge_state(ctx);
		cubs2_topic_rc_published();

		cubs2_serial_bridge_send_rc(&ctx->rc);
	}

	return 0;
}
