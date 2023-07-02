/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <synapse/zbus/channels.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socketcan.h>
#include <zephyr/net/socketcan_utils.h>
#include <zephyr/shell/shell.h>

#include "actuator_vesc_can.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern canbus_detail_t g_canbus_details[];

extern actuator_vesc_can_t g_actuator_vesc_cans[];

static const char* module_name = "actuate_vesc_can";
static synapse_msgs_Actuators g_actuators = synapse_msgs_Actuators_init_default;

static void listener_actuate_vesc_can_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(synapse_msgs_Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuate_vesc_can, listener_actuate_vesc_can_callback);

static int stop_canbus(const actuator_vesc_can_t* actuator)
{
    int err = can_stop(actuator->device);
    if (err != 0) {
        printf("%s: can%d - failed to stop (%d)\n", module_name,
            actuator->bus_id, err);
        g_canbus_details[actuator->bus_id].ready = false;
        return err;
    }
    return err;
}

static void initialize_canbus(const actuator_vesc_can_t* vesc_canbus_init)
{
    enum can_state state;
    struct can_bus_err_cnt err_cnt;
    int err = 0;

    if (g_canbus_details[vesc_canbus_init->bus_id].ready)
        return;

    // check if device ready
    if (!device_is_ready(vesc_canbus_init->device)) {
        printf("%s: can%d - device not ready\n", module_name, vesc_canbus_init->bus_id);
        g_canbus_details[vesc_canbus_init->bus_id].ready = false;
        return;
    }

    // check state
    err = can_get_state(vesc_canbus_init->device, &state, &err_cnt);
    if (err != 0) {
        printf("%s: can%d - failed to get CAN controller state (%d)\n", module_name,
            vesc_canbus_init->bus_id, err);
        g_canbus_details[vesc_canbus_init->bus_id].ready = false;
        return;
    }

    // set device mode
    if (vesc_canbus_init->fd) {
        if (state != CAN_STATE_STOPPED) {
            stop_canbus(vesc_canbus_init);
        }
        err = can_set_mode(vesc_canbus_init->device, CAN_MODE_FD);
        if (err != 0) {
            printf("%s: can%d - set mode FD failed (%d)\n", module_name,
                vesc_canbus_init->bus_id, err);
            stop_canbus(vesc_canbus_init);
            return;
        }
    }

    // start device
    if ((state == CAN_STATE_STOPPED) || (state == CAN_STATE_BUS_OFF)) {

        err = can_start(vesc_canbus_init->device);
        if (err != 0) {
            printf("%s: can%d - start failed\n", module_name,
                vesc_canbus_init->bus_id);
            stop_canbus(vesc_canbus_init);
            return;
        }
    }

    g_canbus_details[vesc_canbus_init->bus_id].ready = true;
    printf("%s: can%d - connected and properly initialized.\n", module_name,
        vesc_canbus_init->bus_id);
    return;
}

void actuate_vesc_can_entry_point()
{
    int err = 0;
    for (int i = 0; i < CONFIG_VESC_CAN_NUMBER; i++) {
        actuator_vesc_can_t vesc_can = g_actuator_vesc_cans[i];
        k_usleep(1e6 / 1);
        initialize_canbus(&vesc_can);
    }

    while (true) {

        // sleep to set control rate at 50 Hz
        k_usleep(1e6 / 50);

        if (g_actuators.velocity_count < 1)
            continue;

        for (int i = 0; i < CONFIG_VESC_CAN_NUMBER; i++) {
            actuator_vesc_can_t vesc_can = g_actuator_vesc_cans[i];
            if (!g_canbus_details[vesc_can.bus_id].ready) {
                initialize_canbus(&vesc_can);
                continue;
            }
            struct can_frame frame = {
                .dlc = can_bytes_to_dlc(4),
                frame.flags = CAN_FRAME_IDE,
            };
            // if (vesc_can.fd) {
            //     frame.flags = CAN_FRAME_FDF | CAN_FRAME_IDE;
            // }
            int32_t erpm = vesc_can.pole_pair * g_actuators.velocity[vesc_can.index] * 60 / (2 * M_PI);
            frame.id = 768 + vesc_can.id;
            frame.data[0] = erpm >> 24 & 255;
            frame.data[1] = erpm >> 16 & 255;
            frame.data[2] = erpm >> 8 & 255;
            frame.data[3] = erpm & 255;

            // send can data
            err = can_send(vesc_can.device, &frame, K_NO_WAIT, NULL, NULL);
            if (err != 0) {
                g_canbus_details[vesc_can.bus_id].ready = false;
                printf("%s: can%d - send failed to VESC ID: %d (%d)\n", module_name,
                    vesc_can.bus_id, vesc_can.id, err);
                continue;
            }
        }
    }
}

K_THREAD_DEFINE(actuate_vesc_can_thread, MY_STACK_SIZE,
    actuate_vesc_can_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
