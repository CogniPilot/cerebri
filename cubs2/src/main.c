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

enum {
	FW_P_X = 190,
	FW_P_Y = 191,
	FW_P_Z = 192,
	FW_P_ROLL = 193,
	FW_P_PITCH = 194,
	FW_P_YAW = 195,
	FW_P_AILERON = 196,
	FW_P_ELEVATOR = 197,
	FW_P_THROTTLE = 198,
	FW_P_RUDDER = 199,
	FW_P_STABILIZER = 200,
	FW_P_CURRENT_WP = 205,
	FW_P_AIRBORNE = 206,
};

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

static void fixed_wing_bridge_init_parameters(CubControl_FixedWingOuterLoop_t *m)
{
	static const real_t params[] = {
		[0] = 0.01,
		[1] = 9.81,
		[2] = 6.0,
		[3] = -4.0, [4] = -5.0, [5] = 3.0,
		[6] = -3.0, [7] = 2.0, [8] = 3.0,
		[9] = 16.20, [10] = 2.0, [11] = 3.0,
		[12] = 16.0, [13] = -4.22, [14] = 3.0,
		[15] = 6.88, [16] = -5.1, [17] = 3.0,
		[18] = -4.0, [19] = -5.0, [20] = 3.0,
		[21] = 10.0,
		[22] = 5.0,
		[23] = 4.0,
		[24] = 1.50,
		[25] = 1.0,    /* lookaheadMin */
		[26] = 5.0,    /* lookaheadMax */
		[27] = 3.0,    /* vCruise */
		[28] = 0.4,    /* takeoffAltitude */
		[29] = 0.7,    /* takeoffThrottleMin */
		[30] = 2.0,    /* takeoffThrottleRate */
		[31] = 0.5,    /* takeoffSpeed */
		[32] = -0.02,  /* takeoffElevDown */
		[33] = 0.15,   /* takeoffElevUp */
		[34] = 0.40,   /* takeoffElevRate */
		[35] = 0.057,  /* mass */
		[36] = 3.5,    /* trimThrust */
		[37] = 0.20,   /* trimElev */
		[38] = 0.0,    /* trimRud */
		[39] = 0.0,    /* trimAil */
		[40] = 7.5,    /* thrMax */
		[41] = 0.01,   /* K_thrustp */
		[42] = 0.4215, /* K_thrusti */
		[43] = 0.075,  /* K_pitchp */
		[44] = 0.216,  /* K_pitchi */
		[45] = 0.107,  /* K_elevp */
		[46] = 0.2107, /* K_elevi */
		[47] = 0.2,    /* K_q */
		[48] = 2.5,    /* K_phi_elev */
		[49] = 0.4,    /* K_deltap */
		[50] = 0.15,   /* K_deltai */
		[51] = 0.10,   /* K_deltad */
		[52] = 0.3,    /* pitchIntegralMax */
		[53] = 7.5,    /* normEsDotIntegralMax */
		[54] = 7.5,    /* distTermIntegralMax */
		[55] = 0.4,    /* rIntegralMax */
		[56] = 0.2,    /* rollIntegralMax */
		[57] = 0.25,   /* K_rollp */
		[58] = 0.10,   /* K_rolli */
		[59] = 1.20,   /* kChi */
		[60] = 30.0 * 3.141592653589793 / 180.0,   /* phiLim */
		[61] = 90.0 * 3.141592653589793 / 180.0,   /* phiDotLim */
		[62] = 1.0 * 3.141592653589793 / 180.0,    /* chiDeadband */
		[63] = 20.0 * 3.141592653589793 / 180.0,   /* phiStickLimit */
	};

	for (size_t i = 0U; i < (sizeof(params) / sizeof(params[0])); i++) {
		m->p[i] = params[i];
	}
	m->p[FW_P_THROTTLE] = 0.7;
	m->p[FW_P_STABILIZER] = 1900.0;
	m->p[FW_P_CURRENT_WP] = 1.0;
	m->p[CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN - 1] = 1.0; /* enable simulation computation */
}

static void fixed_wing_bridge_map_input(CubControl_FixedWingOuterLoop_t *m, const struct control_context *ctx)
{
	float roll = ctx->accel.x;
	float pitch = ctx->accel.y;
	float yaw = ctx->accel.z;

	if (ctx->mocap.valid) {
		quat_to_euler(&ctx->mocap, &roll, &pitch, &yaw);
		m->p[FW_P_X] = ctx->mocap.x;
		m->p[FW_P_Y] = ctx->mocap.y;
		m->p[FW_P_Z] = ctx->mocap.z;
	} else {
		m->p[FW_P_X] = ctx->gyro.x;
		m->p[FW_P_Y] = ctx->gyro.y;
		m->p[FW_P_Z] = ctx->gyro.z;
	}

	m->p[FW_P_ROLL] = roll;
	m->p[FW_P_PITCH] = pitch;
	m->p[FW_P_YAW] = yaw;
}

static void fixed_wing_bridge_map_output(const CubControl_FixedWingOuterLoop_t *m, synapse_topic_RcChannels16_t *rc)
{
	rc->ch0 = pwm_from_centered_stick((float)m->p[FW_P_AILERON], false);
	rc->ch1 = pwm_from_centered_stick((float)m->p[FW_P_ELEVATOR], true);
	rc->ch2 = pwm_from_throttle((float)m->p[FW_P_THROTTLE]);
	rc->ch3 = pwm_from_centered_stick((float)m->p[FW_P_RUDDER], false);
	rc->ch4 = (int32_t)clampf_local((float)m->p[FW_P_STABILIZER], 1000.0f, 2000.0f);
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	int rc;

	*ctx = (struct control_context){0};
	startup(&g_model);
	fixed_wing_bridge_init_parameters(&g_model);

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
		// Wait for next telemetry packet or 100Hz trigger
		cubs2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status, &ctx->dt);
		ctx->now_ms = k_uptime_get();

		// Fetch latest mocap data from the bridge
		cubs2_serial_bridge_get_mocap(&ctx->mocap);

		// Update model inputs
		fixed_wing_bridge_map_input(&g_model, ctx);

		// Step the eFMU
		dostep(&g_model, (real_t)ctx->dt);

		// Map model outputs to RC sticks
		fixed_wing_bridge_map_output(&g_model, &ctx->rc);
		LOG_INF("ail=%.2f elev=%.2f thr=%.2f rud=%.2f stab=%.2f d=%d z=%.2f yaw=%.2f air=%d",
			(double)g_model.p[FW_P_AILERON], (double)g_model.p[FW_P_ELEVATOR],
			(double)g_model.p[FW_P_THROTTLE], (double)g_model.p[FW_P_RUDDER],
			(double)g_model.p[FW_P_STABILIZER],
			(int)g_model.p[FW_P_CURRENT_WP],
			(double)g_model.p[FW_P_Z], (double)g_model.p[FW_P_YAW],
			(int)g_model.p[FW_P_AIRBORNE]);

		// Publish stick overrides to the bridge output
		publish_bridge_state(ctx);
		cubs2_topic_rc_published();

		// Send RC overrides back to Ubuntu bridge
		cubs2_serial_bridge_send_rc(&ctx->rc);
	}

	return 0;
}
