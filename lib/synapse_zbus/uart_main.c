/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY -1

#define RX_BUF_SIZE 1024

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

static TinyFrame* g_tf = NULL;

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

// ROS -> cerebri
TOPIC_LISTENER(in_actuators, Actuators)
TOPIC_LISTENER(in_bezier_trajectory, BezierTrajectory)
TOPIC_LISTENER(in_cmd_vel, Twist)
TOPIC_LISTENER(in_joy, Joy)
TOPIC_LISTENER(in_odometry, Odometry)

void listener_synapse_zbus_uart_callback(const struct zbus_channel* chan)
{
    if (chan == NULL) { // start of if else statements for channel type
    }
    // cerebri -> ROS
    TOPIC_PUBLISHER(out_actuators, Actuators, SYNAPSE_OUT_ACTUATORS_TOPIC)
    TOPIC_PUBLISHER(out_odometry, Odometry, SYNAPSE_OUT_ODOMETRY_TOPIC)
}

ZBUS_LISTENER_DEFINE(listener_synapse_zbus_uart, listener_synapse_zbus_uart_callback);

static const struct device* const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static void write_uart(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

static void uart_entry_point(void)
{
    // Set up the TinyFrame library
    g_tf = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    g_tf->write = write_uart;

    // ros -> cerebri
    TF_AddGenericListener(g_tf, genericListener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_ACTUATORS_TOPIC, in_actuators_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);

    while (true) {
        uint8_t c;
        int count = 0;
        while (uart_poll_in(uart_dev, &c) == 0) {
            TF_AcceptChar(g_tf, c);
            count++;
        }
        TF_Tick(g_tf);
    }
}

K_THREAD_DEFINE(synapse_zbus_uart, MY_STACK_SIZE, uart_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
