/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"
#include "hotpath_memory.h"
#include "topic_shell.h"
#include "generated_fixed_wing/CubControl_FixedWingOuterLoop.h"

#include <stdbool.h>
#include <stdint.h>

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

/* User: Map incoming telemetry to Modelica parameters p[305] */
static void fixed_wing_bridge_map_input(CubControl_FixedWingOuterLoop_t *m, const struct control_context *ctx)
{
	// Example mapping (indices are placeholders):
	// m->p[0] = ctx->attitude.roll;
	// m->p[1] = ctx->attitude.pitch;
	// m->p[2] = ctx->attitude.yaw;
	// m->p[3] = ctx->gyro.x;
}

/* User: Map Modelica parameters/outputs to RC channel overrides */
static void fixed_wing_bridge_map_output(const CubControl_FixedWingOuterLoop_t *m, synapse_topic_RcChannels16_t *rc)
{
	// Example mapping (indices are placeholders):
	// rc->ch0 = (int)m->p[50]; // Aileron
	// rc->ch1 = (int)m->p[51]; // Elevator
	// rc->ch2 = (int)m->p[52]; // Throttle
	// rc->ch3 = (int)m->p[53]; // Rudder
}

int main(void)
{
	struct control_context *const ctx = &g_control_ctx;
	int rc;

	*ctx = (struct control_context){0};
	startup(&g_model);

	rc = cubs2_control_io_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("CUBS2 Fixed-Wing Bridge starting");

	while (true) {
		// Wait for next telemetry packet or 100Hz trigger
		cubs2_control_input_wait(&ctx->gyro, &ctx->accel, &ctx->rc, &ctx->status, &ctx->dt);
		ctx->now_ms = k_uptime_get();

		// Update model inputs
		fixed_wing_bridge_map_input(&g_model, ctx);

		// Step the eFMU
		dostep(&g_model, (real_t)ctx->dt);

		// Map model outputs to RC sticks
		fixed_wing_bridge_map_output(&g_model, &ctx->rc);

		// Publish stick overrides to the bridge output
		publish_bridge_state(ctx);
		cubs2_topic_rc_published();
	}

	return 0;
}
