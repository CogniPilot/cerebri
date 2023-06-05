/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#if defined(CONFIG_SYNAPSE_ZBUS_UART)

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

#define RX_BUF_SIZE 100

static const struct device* const uart_dev = DEVICE_DT_GET(DT_ALIAS(telem1));

static void write_uart(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

static TinyFrame g_tf = {
    .write = write_uart,
    .peer_bit = TF_MASTER
};

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
#if !defined(CONFIG_ARCH_POSIX)
void serial_cb(const struct device* dev, void* user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        TF_AcceptChar(&g_tf, c);
    }
}
#endif

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

static void uart_entry_point(void)
{
    // ros -> cerebri
    TF_AddGenericListener(&g_tf, genericListener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_ACTUATORS_TOPIC, in_actuators_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);

#if defined(CONFIG_ARCH_POSIX)
    while (true) {
        uint8_t c;
        int count = 0;
        while (uart_poll_in(uart_dev, &c) == 0) {
            TF_AcceptChar(&g_tf, c);
            count++;
        }
        TF_Tick(&g_tf);
    }
#else // CONFIG_ARCH_POSIX
    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return;
    }
    uart_irq_rx_enable(uart_dev);

    while (true) {
        k_msleep(1000);
        TF_Tick(&g_tf);
    }
#endif // CONFIG_ARCH_POSIX
}

K_THREAD_DEFINE(synapse_zbus_uart, MY_STACK_SIZE, uart_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

#endif // defined(CONFIG_SYNAPSE_ZBUS_UART)
/* vi: ts=4 sw=4 et */
