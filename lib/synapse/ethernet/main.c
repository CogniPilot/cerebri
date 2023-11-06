/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include <fcntl.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cerebri/synapse/zbus/common.h>

#define TOPIC_LISTENER(CHANNEL, CLASS)                                           \
    static TF_Result CHANNEL##_Listener(TinyFrame* tf, TF_Msg* frame)            \
    {                                                                            \
        CLASS msg = CLASS##_init_default;                                        \
        pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);   \
        int status = pb_decode(&stream, CLASS##_fields, &msg);                   \
        if (status) {                                                            \
            zbus_chan_pub(&chan_##CHANNEL, &msg, K_FOREVER);                     \
        } else {                                                                 \
            printf("%s decoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                        \
        return TF_STAY;                                                          \
    }

#define TOPIC_PUBLISHER(CHANNEL, CLASS, TOPIC)                                   \
    else if (chan == &chan_##CHANNEL)                                            \
    {                                                                            \
        TF_Msg msg;                                                              \
        TF_ClearMsg(&msg);                                                       \
        uint8_t buf[CLASS##_size];                                               \
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));     \
        int status = pb_encode(&stream, CLASS##_fields, chan->message);          \
        if (status) {                                                            \
            msg.type = TOPIC;                                                    \
            msg.data = buf;                                                      \
            msg.len = stream.bytes_written;                                      \
            TF_Send(&g_tf, &msg);                                                \
        } else {                                                                 \
            printf("%s encoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                        \
    }

LOG_MODULE_REGISTER(synapse_ethernet, CONFIG_CEREBRI_SYNAPSE_ETHERNET_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 5

#define RX_BUF_SIZE 1024

#define BIND_PORT 4242

static volatile int g_client = -1;
static int error_count = 0;

static void write_ethernet(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    if (g_client < 0) {
        return;
    }

    int out_len;
    const char* p;
    p = buf;
    do {
        out_len = zsock_send(g_client, p, len, 0);
        if (out_len < 0) {
            LOG_ERR("send: %d\n", errno);
            if (error_count++ > 10) {
                // trigger reconnect
                g_client = -1;
            }
            return;
        } else {
            // reset error count
            error_count = 0;
        }
        p += out_len;
        len -= out_len;
    } while (len);
}

static TinyFrame g_tf = { 0 };

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

// ROS -> cerebri
TOPIC_LISTENER(actuators, synapse_msgs_Actuators)
TOPIC_LISTENER(altimeter, synapse_msgs_Altimeter)
TOPIC_LISTENER(battery_state, synapse_msgs_Altimeter)
TOPIC_LISTENER(bezier_trajectory, synapse_msgs_BezierTrajectory)
TOPIC_LISTENER(cmd_vel, synapse_msgs_Twist)
TOPIC_LISTENER(imu, synapse_msgs_Imu)
TOPIC_LISTENER(joy, synapse_msgs_Joy)
TOPIC_LISTENER(led_array, synapse_msgs_LEDArray)
TOPIC_LISTENER(magnetic_field, synapse_msgs_MagneticField)
TOPIC_LISTENER(nav_sat_fix, synapse_msgs_NavSatFix)
TOPIC_LISTENER(odometry, synapse_msgs_Odometry)
TOPIC_LISTENER(wheel_odometry, synapse_msgs_WheelOdometry)

void listener_synapse_ethernet_callback(const struct zbus_channel* chan)
{
    // cerebri -> ROS
    if (chan == NULL) { } // start of if else statements for channel type
    TOPIC_PUBLISHER(actuators, synapse_msgs_Actuators, SYNAPSE_ACTUATORS_TOPIC)
    TOPIC_PUBLISHER(odometry, synapse_msgs_Odometry, SYNAPSE_ODOMETRY_TOPIC)
    // TOPIC_PUBLISHER(out_wheel_odometry, synapse_msgs_WheelOdometry, SYNAPSE_WHEEL_ODOMETRY_TOPIC)
}

ZBUS_LISTENER_DEFINE(listener_synapse_ethernet, listener_synapse_ethernet_callback);
ZBUS_CHAN_ADD_OBS(chan_actuators, listener_synapse_ethernet, 1);
ZBUS_CHAN_ADD_OBS(chan_odometry, listener_synapse_ethernet, 1);
// ZBUS_CHAN_ADD_OBS(chan_wheel_odometry, listener_synapse_ethernet, 1);

static bool set_blocking_enabled(int fd, bool blocking)
{
    if (fd < 0)
        return false;
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return false;
    flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
    return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
}

static void ethernet_entry_point(void)
{
    static uint8_t rx1_buf[RX_BUF_SIZE];

    int serv;
    struct sockaddr_in bind_addr;
    static int counter;

    TF_InitStatic(&g_tf, TF_MASTER, write_ethernet);

    serv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    set_blocking_enabled(serv, true);

    if (serv < 0) {
        LOG_ERR("socket: %d", errno);
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (zsock_bind(serv, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        LOG_ERR("bind: %d", errno);
        exit(1);
    }

    if (zsock_listen(serv, 5) < 0) {
        LOG_ERR("listen: %d", errno);
        exit(1);
    }

    // ros -> cerebri
    TF_AddGenericListener(&g_tf, genericListener);
    TF_AddTypeListener(&g_tf, SYNAPSE_ACTUATORS_TOPIC, actuators_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_ALTIMETER_TOPIC, altimeter_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_BATTERY_STATE_TOPIC, battery_state_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_BEZIER_TRAJECTORY_TOPIC, bezier_trajectory_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_CMD_VEL_TOPIC, cmd_vel_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IMU_TOPIC, imu_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_JOY_TOPIC, joy_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_LED_ARRAY_TOPIC, led_array_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_MAGNETIC_FIELD_TOPIC, magnetic_field_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_NAV_SAT_FIX_TOPIC, nav_sat_fix_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_ODOMETRY_TOPIC, odometry_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_WHEEL_ODOMETRY_TOPIC, wheel_odometry_Listener);

    // ROS -> cerebri (Hardware-in-the-loop only)

    while (1) {
        LOG_INF("socket waiting for connection on port: %d", BIND_PORT);
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        g_client = zsock_accept(serv, (struct sockaddr*)&client_addr,
            &client_addr_len);
        k_msleep(1000);

        if (g_client < 0) {
            continue;
        }

        zsock_inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        LOG_INF("connection #%d from %s", counter++, addr_str);

        while (1) {
            k_msleep(1);
            if (g_client < 0) {
                LOG_ERR("no client, triggering reconnect");
                break;
            }
            int len = zsock_recv(g_client, rx1_buf, sizeof(rx1_buf), 0);
            if (len < 0) {
                continue;
            }
            TF_Accept(&g_tf, rx1_buf, len);
            TF_Tick(&g_tf);
        }
    }
}

K_THREAD_DEFINE(synapse_ethernet, MY_STACK_SIZE, ethernet_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
