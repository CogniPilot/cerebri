/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mixing.h"

void b3rb_set_actuators(synapse_pb_Actuators *msg, double turn_angle, double omega_fwd, bool armed)
{
	msg->has_stamp = true;
	stamp_msg(&msg->stamp, k_uptime_ticks());

	if (!armed) {
		// stop if not armed
		turn_angle = 0;
		omega_fwd = 0;
	}

	msg->position_count = 1;
	msg->position[0] = turn_angle;

	msg->velocity_count = 1;
	msg->velocity[0] = omega_fwd;

	msg->normalized_count = 1;
	msg->normalized[0] = armed ? 1 : -1;
}

/* vi: ts=4 sw=4 et */
