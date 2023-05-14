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

#include <pb_decode.h>
#include <pb_encode.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>
#include <synapse_tinyframe/utils.h>

#include <synapse_zbus/channels.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY -1

#define TX_BUF_SIZE 1024
#define RX_BUF_SIZE 1024

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

static TinyFrame* g_tf = NULL;

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

#define TOPIC_LISTENER(CHANNEL, CLASS)                                         \
    static TF_Result CHANNEL##_Listener(TinyFrame* tf, TF_Msg* frame)          \
    {                                                                          \
        CLASS msg = CLASS##_init_zero;                                         \
        pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len); \
        int status = pb_decode(&stream, CLASS##_fields, &msg);                 \
        if (status) {                                                          \
            zbus_chan_pub(&chan_##CHANNEL, &msg, K_FOREVER);                   \
        } else {                                                               \
            printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));            \
        }                                                                      \
        return TF_STAY;                                                        \
    }

TOPIC_LISTENER(in_bezier_trajectory, BezierTrajectory);
TOPIC_LISTENER(in_cmd_vel, Twist);
TOPIC_LISTENER(in_joy, Joy);
TOPIC_LISTENER(in_odometry, Odometry);


void listener_synapse_zbus_uart_callback(const struct zbus_channel *chan) {
    if (chan == &chan_out_actuators) {
        TF_Msg msg;
        TF_ClearMsg(&msg);
        uint8_t buf[500];
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
        int status = pb_encode(&stream, Actuators_fields, chan->message);
        if (status) {
            msg.type = SYNAPSE_OUT_ACTUATORS_TOPIC;
            msg.data = buf;
            msg.len =  stream.bytes_written;
            TF_Send(g_tf, &msg);
        } else {
            printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        }
    }
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
    uint8_t tx0_buf[TX_BUF_SIZE];

    TF_Msg msg;

    // Set up the TinyFrame library
    g_tf = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    g_tf->write = write_uart;

    TF_AddGenericListener(g_tf, genericListener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(g_tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);

    while (true) {
        // send cmd vel topic
        {
            Twist message = Twist_init_zero;
            pb_ostream_t tx_stream = pb_ostream_from_buffer(tx0_buf, Twist_size);
            pb_encode(&tx_stream, Twist_fields, &message);

            TF_ClearMsg(&msg);
            msg.type = SYNAPSE_OUT_CMD_VEL_TOPIC;
            msg.len = tx_stream.bytes_written;
            msg.data = (pu8)tx0_buf;
            TF_Send(g_tf, &msg);
        }

        // receive messages
        {
            uint8_t c;
            int count = 0;
            while (uart_poll_in(uart_dev, &c) == 0) {
                TF_AcceptChar(g_tf, c);
                count++;
            }
        }

        // sleep 0.01 seconds
        // k_busy_wait(10000);

        // should move TF tick to a clock thread
        TF_Tick(g_tf);
    }
}

K_THREAD_DEFINE(synapse_zbus_uart, MY_STACK_SIZE, uart_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
