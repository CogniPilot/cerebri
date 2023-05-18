/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>

#include <stdio.h>

#include <synapse_zbus/channels.h>

double thrust = 0;
double yaw = 0;

void listener_control_rover_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_joy) {
        Joy* msg = (Joy*)(chan->message);
        thrust = msg->axes[1];
        yaw = msg->axes[3];
    }
}

ZBUS_LISTENER_DEFINE(listener_control_rover, listener_control_rover_callback);

void control_entry_point(void* p1, void* p2, void* p3)
{
    while (true) {
        // send data to motors
        Actuators msg = Actuators_init_zero;
        msg.has_header = true;
        strncpy(msg.header.frame_id, "test", 16);
        msg.header.has_stamp = true;
        msg.header.seq = 0;
        msg.header.stamp.nanos = 0;
        msg.header.stamp.seconds = 0;
        msg.normalized_count = 0;
        msg.position_count = 0;
        msg.velocity[0] = 1.0 * thrust + 1.0 * yaw;
        msg.velocity[1] = 1.0 * thrust - 1.0 * yaw;
        msg.velocity[2] = 1.0 * thrust - 1.0 * yaw;
        msg.velocity[3] = 1.0 * thrust + 1.0 * yaw;
        msg.velocity_count = 4;
        zbus_chan_pub(&chan_out_actuators, &msg, K_NO_WAIT);

        // sleep to set control rate at 10 Hz
        k_usleep(1e6 / 10);
    }
}

K_THREAD_DEFINE(control_thread, 500,
    control_entry_point, NULL, NULL, NULL,
    -1, 0, 0);

/* vi: ts=4 sw=4 et */
