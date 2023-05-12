/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
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

#define BIND_PORT 4242

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

/*
static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}
*/

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
TOPIC_LISTENER(out_actuators, Actuators);

#if defined(CONFIG_SYNAPSE_ZBUS_UART)
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
    TinyFrame* tf;
    tf = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    tf->write = write_uart;

    // TF_AddGenericListener(tf, genericListener);
    TF_AddTypeListener(tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);
    TF_AddTypeListener(tf, SYNAPSE_OUT_ACTUATORS_TOPIC, out_actuators_Listener);

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
            TF_Send(tf, &msg);
        }

        // receive messages
        {
            uint8_t c;
            int count = 0;
            while (uart_poll_in(uart_dev, &c) == 0) {
                TF_AcceptChar(tf, c);
                count++;
            }
        }

        // sleep 0.01 seconds
        // k_busy_wait(10000);

        // should move TF tick to a clock thread
        TF_Tick(tf);
    }
}

K_THREAD_DEFINE(synapse_zbus_uart, MY_STACK_SIZE, uart_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);
#endif

#if defined(CONFIG_SYNAPSE_ZBUS_ETHERNET)
static void write_ethernet(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    int client = *(int*)(tf->userdata);
    int out_len;
    const char* p;
    p = buf;
    do {
        out_len = send(client, p, len, 0);
        if (out_len < 0) {
            printf("error: send: %d\n", errno);
            return;
        }
        p += out_len;
        len -= out_len;
    } while (len);
}

static void ethernet_entry_point(void)
{
    static uint8_t rx1_buf[RX_BUF_SIZE];

    int serv;
    struct sockaddr_in bind_addr;
    static int counter;

    serv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (serv < 0) {
        printf("error: socket: %d\n", errno);
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (bind(serv, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("error: bind: %d\n", errno);
        exit(1);
    }

    if (listen(serv, 5) < 0) {
        printf("error: listen: %d\n", errno);
        exit(1);
    }

    printf("TCP server waits for a connection on "
           "port %d...\n",
        BIND_PORT);

    // Set up the TinyFrame library
    static TinyFrame* tf;
    tf = TF_Init(TF_MASTER); // 1 = master, 0 = slave
    tf->write = write_ethernet;

    // TF_AddGenericListener(tf, genericListener);
    TF_AddTypeListener(tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);
    TF_AddTypeListener(tf, SYNAPSE_OUT_ACTUATORS_TOPIC, out_actuators_Listener);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        int client = accept(serv, (struct sockaddr*)&client_addr,
            &client_addr_len);
        tf->userdata = &client;

        if (client < 0) {
            printf("error: accept: %d\n", errno);
            continue;
        } else {
            printf("connected\n");
        }

        inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        printf("Connection #%d from %s\n", counter++, addr_str);

        while (1) {
            // send cmd vel topic
            /*
            {
                TF_Msg frame;
                Twist msg = Twist_init_zero;
                msg.has_linear = true;
                msg.linear.x = 1;
                msg.linear.y = 2;
                msg.linear.z = 3;
                msg.has_angular = true;
                msg.angular.x = 4;
                msg.angular.y = 5;
                msg.angular.z = 6;
                pb_ostream_t tx_stream = pb_ostream_from_buffer(tx1_buf, Twist_size);
                pb_encode(&tx_stream, Twist_fields, &msg);

                TF_ClearMsg(&frame);
                frame.type = SYNAPSE_OUT_CMD_VEL_TOPIC;
                frame.len = tx_stream.bytes_written;
                frame.data = (pu8)tx1_buf;
                TF_Send(tf, &frame);
            }
            */

            // receive messages
            {
                int len = recv(client, rx1_buf, sizeof(rx1_buf), 0);
                TF_Accept(tf, rx1_buf, len);
                // printf("len: %d\n", len);
            }

            // should move tf tick to a clock thread
            TF_Tick(tf);
        }
    }
}

K_THREAD_DEFINE(synapse_zbus_ethernet, MY_STACK_SIZE, ethernet_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);
#endif

/* vi: ts=4 sw=4 et */
