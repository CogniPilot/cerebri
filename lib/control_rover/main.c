/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/kernel.h>

#include <stdio.h>

#include <synapse_zbus/channels.h>

#define MY_STACK_SIZE 10240
#define MY_PRIORITY 4

static Odometry g_odometry = Odometry_init_zero;
static Joy g_joy = Joy_init_zero;
static Twist g_cmd_vel = Twist_init_zero;

static void listener_control_rover_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_joy) {
        g_joy = *(Joy*)(chan->message);
    } else if (chan == &chan_in_odometry) {
        g_odometry = *(Odometry*)(chan->message);
    } else if (chan == &chan_in_cmd_vel) {
        g_cmd_vel = *(Twist*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_control_rover, listener_control_rover_callback);

static void control_entry_point(void* p1, void* p2, void* p3)
{
    int mode = 0;
    double scale = 30.0;
    static Actuators msg = Actuators_init_zero;
    msg.velocity_count = 4;
    msg.has_header = true;
    strncpy(msg.header.frame_id, "test", 16);
    msg.header.has_stamp = true;
    msg.header.seq = 0;
    msg.header.stamp.nanos = 0;
    msg.header.stamp.seconds = 0;
    msg.normalized_count = 0;
    msg.position_count = 0;

    while (true) {
        if (mode == 0) {
            // joystick mode
            double thrust = g_joy.axes[1];
            double yaw = g_joy.axes[3];
            msg.velocity[0] = scale * (1.0 * thrust + 1.0 * yaw);

            msg.velocity[1] = scale * (1.0 * thrust - 1.0 * yaw);
            msg.velocity[2] = scale * (1.0 * thrust - 1.0 * yaw);
            msg.velocity[3] = scale * (1.0 * thrust + 1.0 * yaw);
        } else if (mode == 1) {
            // cmd_vel mode
            double thrust = g_cmd_vel.linear.x;
            double yaw = g_cmd_vel.angular.z;
            msg.velocity[0] = scale * (1.0 * thrust + 1.0 * yaw);
            msg.velocity[1] = scale * (1.0 * thrust - 1.0 * yaw);
            msg.velocity[2] = scale * (1.0 * thrust - 1.0 * yaw);
            msg.velocity[3] = scale * (1.0 * thrust + 1.0 * yaw);
        }

        // sleep to set control rate at 10 Hz
        zbus_chan_pub(&chan_out_actuators, &msg, K_SECONDS(1));
        k_msleep(1e3 / 10);
    }
}

K_THREAD_DEFINE(control_thread, MY_STACK_SIZE,
    control_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
