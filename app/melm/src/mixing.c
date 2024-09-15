/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mixing.h"

void melm_set_actuators(synapse_pb_Actuators *msg, double omega_left, double omega_right, bool armed)
{
	msg->has_stamp = true;
	stamp_msg(&msg->stamp, k_uptime_ticks());

	if (!armed) {
		// stop if not armed
		omega_left = 0;
		omega_right = 0;
	}

	msg->velocity_count = 2;
	msg->velocity[0] = omega_left;
	msg->velocity[1] = omega_right;
}

/* vi: ts=4 sw=4 et */
