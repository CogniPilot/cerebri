/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <synapse/zbus/channels.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

static Actuators g_actuators = Actuators_init_zero;

static void listener_actuator_vesc_can_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_vesc_can, listener_actuator_vesc_can_callback);

void vesc_can_entry_point(const struct shell* sh)
{

    while (true) {

        shell_print(sh, "MADE IT VESC!!!!");

        // sleep to set control rate at 50 Hz
        k_usleep(1e6 / 10);
    }
}

K_THREAD_DEFINE(can_vesc_thread, MY_STACK_SIZE,
    vesc_can_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
