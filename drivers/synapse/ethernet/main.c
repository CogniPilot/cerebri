/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#include <fcntl.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "synapse/zbus/common.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 5

#define RX_BUF_SIZE 100

#define BIND_PORT 4242

static const char* module_name = "synapse_ethernet";
static volatile int g_client = -1;

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
            printf("%s: error: send: %d\n", module_name, errno);
            // trigger reconnect
            g_client = -1;
            return;
        }
        p += out_len;
        len -= out_len;
    } while (len);
}

static TinyFrame g_tf = {
    .write = write_ethernet,
    .peer_bit = TF_MASTER,
};

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    dumpFrameInfo(msg);
    return TF_STAY;
}

// ROS -> cerebri
TOPIC_LISTENER(in_actuators, synapse_msgs_Actuators)
TOPIC_LISTENER(in_bezier_trajectory, synapse_msgs_BezierTrajectory)
TOPIC_LISTENER(in_cmd_vel, synapse_msgs_Twist)
TOPIC_LISTENER(in_joy, synapse_msgs_Joy)
TOPIC_LISTENER(in_odometry, synapse_msgs_Odometry)
TOPIC_LISTENER(in_nav_sat_fix, synapse_msgs_NavSatFix)

void listener_synapse_ethernet_callback(const struct zbus_channel* chan)
{
    if (chan == NULL) { // start of if else statements for channel type
    }
    // cerebri -> ROS
    TOPIC_PUBLISHER(out_actuators, synapse_msgs_Actuators, SYNAPSE_OUT_ACTUATORS_TOPIC)
    TOPIC_PUBLISHER(out_odometry, synapse_msgs_Odometry, SYNAPSE_OUT_ODOMETRY_TOPIC)
}

ZBUS_LISTENER_DEFINE(listener_synapse_ethernet, listener_synapse_ethernet_callback);

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

    serv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    set_blocking_enabled(serv, true);

    if (serv < 0) {
        printf("%s: error: socket: %d\n", module_name, errno);
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (zsock_bind(serv, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("%s: error: bind: %d\n", module_name, errno);
        exit(1);
    }

    if (zsock_listen(serv, 5) < 0) {
        printf("%s: error: listen: %d\n", module_name, errno);
        exit(1);
    }

    printf("%s: TCP server waits for a connection on "
           "port %d...\n",
        module_name,
        BIND_PORT);

    // ros -> cerebri
    TF_AddGenericListener(&g_tf, genericListener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_ACTUATORS_TOPIC, in_actuators_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_BEZIER_TRAJECTORY_TOPIC, in_bezier_trajectory_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_CMD_VEL_TOPIC, in_cmd_vel_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_JOY_TOPIC, in_joy_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_ODOMETRY_TOPIC, in_odometry_Listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_NAVSAT_TOPIC, in_nav_sat_fix_Listener);

    while (1) {
        printf("%s: socket waiting for connection on port: %d\n",
            module_name, BIND_PORT);
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
        printf("%s: connection #%d from %s\n",
            module_name, counter++, addr_str);

        while (1) {
            if (g_client < 0) {
                printf("%s: triggering reconnect.\n", module_name);
                break;
            }
            int len = zsock_recv(g_client, rx1_buf, sizeof(rx1_buf), 0);
            if (len < 0) {
                continue;
            }

            TF_Accept(&g_tf, rx1_buf, len);
            TF_Tick(&g_tf);
            k_msleep(1);
        }
    }
}

K_THREAD_DEFINE(synapse_ethernet, MY_STACK_SIZE, ethernet_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);
/* vi: ts=4 sw=4 et */
