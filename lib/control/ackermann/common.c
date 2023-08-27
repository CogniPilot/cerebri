/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cerebri/synapse/zbus/common.h>

void ackermann_set_actuators(synapse_msgs_Actuators* msg, double turn_angle, double omega_fwd)
{
    msg->has_header = true;
    stamp_header(&msg->header, k_uptime_ticks());
    msg->header.seq++;
    strncpy(msg->header.frame_id, "map", sizeof(msg->header.frame_id) - 1);

    msg->position_count = 1;
    msg->velocity_count = 1;
    msg->normalized_count = 2;
    msg->position[0] = turn_angle;
    msg->velocity[0] = omega_fwd;
#ifdef CONFIG_BUGGY3_MOTOR_ENB_REQUIRED
    msg->normalized[0] = -1;
#endif
}

/* vi: ts=4 sw=4 et */
