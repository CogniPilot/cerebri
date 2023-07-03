/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <time.h>
#include <zephyr/kernel.h>

#include <stdio.h>

#include <synapse/zbus/channels.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

// private context
typedef struct ctx_ {
    const char* module_name;
    synapse_msgs_NavSatFix sub_nav_sat_fix;
    synapse_msgs_Imu sub_imu;
    synapse_msgs_MagneticField sub_magnetic_field;
    synapse_msgs_Altimeter sub_altimeter;
    synapse_msgs_Odometry pub_odometry;
} ctx_t;

// private initialization
static ctx_t ctx = {
    .module_name = "estimate_iekf",
    .sub_nav_sat_fix = synapse_msgs_NavSatFix_init_default,
    .sub_imu = synapse_msgs_Imu_init_default,
    .sub_magnetic_field = synapse_msgs_MagneticField_init_default,
    .sub_altimeter = synapse_msgs_Altimeter_init_default,
    .pub_odometry = synapse_msgs_Odometry_init_default
};

void listener_estimate_iekf_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_nav_sat_fix) {
        ctx.sub_nav_sat_fix = *(synapse_msgs_NavSatFix*)(chan->message);
    } else if (chan == &chan_in_imu) {
        ctx.sub_imu = *(synapse_msgs_Imu*)(chan->message);
    } else if (chan == &chan_in_magnetic_field) {
        ctx.sub_magnetic_field = *(synapse_msgs_MagneticField*)(chan->message);
    } else if (chan == &chan_in_altimeter) {
        ctx.sub_altimeter = *(synapse_msgs_Altimeter*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_estimate_iekf, listener_estimate_iekf_callback);

void estimate_iekf_entry_point(void* p1, void* p2, void* p3)
{
    double dt = 1.0 / 50;
    double theta = 0;

    while (true) {
        // sleep to set rate
        k_usleep(dt * 1e6);

        theta += dt * ctx.sub_imu.angular_velocity.z;

        // xyz
        ctx.pub_odometry.has_header = true;
        const char frame_id[] = "map";
        const char child_frame_id[] = "base_link";
        strncpy(ctx.pub_odometry.header.frame_id, frame_id, sizeof(ctx.pub_odometry.header.frame_id));
        strncpy(ctx.pub_odometry.child_frame_id, child_frame_id, sizeof(ctx.pub_odometry.child_frame_id));
        ctx.pub_odometry.has_pose = true;
        ctx.pub_odometry.pose.has_pose = true;
        ctx.pub_odometry.pose.pose.has_orientation = true;
        ctx.pub_odometry.pose.pose.has_position = true;
        ctx.pub_odometry.pose.pose.orientation.x = 0;
        ctx.pub_odometry.pose.pose.orientation.y = 0;
        ctx.pub_odometry.pose.pose.orientation.z = cos(theta / 2.0);
        ctx.pub_odometry.pose.pose.orientation.w = sin(theta / 2.0);
        zbus_chan_pub(&chan_out_odometry, &ctx.pub_odometry, K_NO_WAIT);
    }
}

K_THREAD_DEFINE(estimate_iekf_thread, MY_STACK_SIZE,
    estimate_iekf_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
