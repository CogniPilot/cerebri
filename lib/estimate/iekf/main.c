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

static const char* module_name = "estimate_iekf";

static synapse_msgs_NavSatFix g_nav_sat_fix = synapse_msgs_NavSatFix_init_default;
static synapse_msgs_Imu g_imu = synapse_msgs_Imu_init_default;
static synapse_msgs_MagneticField g_mag = synapse_msgs_MagneticField_init_default;
static synapse_msgs_Altimeter g_alt = synapse_msgs_Altimeter_init_default;

void listener_estimate_iekf_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_nav_sat_fix) {
        g_nav_sat_fix = *(synapse_msgs_NavSatFix*)(chan->message);
    } else if (chan == &chan_in_imu) {
        g_imu = *(synapse_msgs_Imu*)(chan->message);
    } else if (chan == &chan_in_magnetic_field) {
        g_mag = *(synapse_msgs_MagneticField*)(chan->message);
    } else if (chan == &chan_in_altimeter) {
        g_alt = *(synapse_msgs_Altimeter*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_estimate_iekf, listener_estimate_iekf_callback);

void estimate_iekf_entry_point(void* p1, void* p2, void* p3)
{
    while (true) {
        // sleep to set rate
        k_usleep(1e6 / 1);

        // gps
        printf("%s: lat: %15.7f long: %15.7f alt: %15.1f\n",
            module_name,
            g_nav_sat_fix.latitude,
            g_nav_sat_fix.longitude,
            g_nav_sat_fix.altitude);

        // mag
        printf("%s: mag x: %15.7f y: %15.7f z: %15.7f\n",
            module_name,
            g_mag.magnetic_field.x,
            g_mag.magnetic_field.y,
            g_mag.magnetic_field.z);

        // imu
        printf("%s: imu ax: %15.7f ay: %15.7f az: %15.7f gx: %15.7f gy: %15.7f gz: %15.7f\n",
            module_name,
            g_imu.linear_acceleration.x,
            g_imu.linear_acceleration.y,
            g_imu.linear_acceleration.z,
            g_imu.angular_velocity.x,
            g_imu.angular_velocity.y,
            g_imu.angular_velocity.z);

        // altimeter
        printf("%s: alt z: %15.7f\n",
            module_name,
            g_alt.vertical_position);
    }
}

K_THREAD_DEFINE(estimate_iekf_thread, MY_STACK_SIZE,
    estimate_iekf_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
