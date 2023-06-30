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

static synapse_msgs_NavSatFix g_nav_sat_fix = synapse_msgs_NavSatFix_init_zero;

void listener_estimate_iekf_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_nav_sat_fix) {
        g_nav_sat_fix = *(synapse_msgs_NavSatFix*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_estimate_iekf, listener_estimate_iekf_callback);

void estimate_iekf_entry_point(void* p1, void* p2, void* p3)
{
    while (true) {
        printf("%s: lat: %15.7f long: %15.7f alt: %15.1f\n",
            module_name,
            g_nav_sat_fix.latitude,
            g_nav_sat_fix.longitude,
            g_nav_sat_fix.altitude);
        // sleep to set control rate at 50 Hz
        k_usleep(1e6 / 1);
    }
}

K_THREAD_DEFINE(estimate_iekf_thread, MY_STACK_SIZE,
    estimate_iekf_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
