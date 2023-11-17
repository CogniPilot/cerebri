/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cerebri/synapse/zbus/common.h>

void diffdrive_set_actuators(synapse_msgs_Actuators* msg, double omega_fwd, double omega_turn)
{
    msg->has_header = true;
    stamp_header(&msg->header, k_uptime_ticks());
    msg->header.seq++;
    strncpy(msg->header.frame_id, "odom", sizeof(msg->header.frame_id) - 1);

    msg->velocity_count = 4;
    msg->velocity[0] = omega_fwd + omega_turn;
    msg->velocity[1] = omega_fwd - omega_turn;
    msg->velocity[2] = omega_fwd - omega_turn;
    msg->velocity[3] = omega_fwd + omega_turn;
}

/* vi: ts=4 sw=4 et */
