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

LOG_MODULE_REGISTER(synapse_ethernet, CONFIG_CEREBRI_SYNAPSE_ETHERNET_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 5

#define RX_BUF_SIZE 1024
#define BIND_PORT 4242

typedef struct context_s {
    syn_node_t node;
    synapse_msgs_Actuators actuators;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Odometry estimator_odometry;
    synapse_msgs_Fsm fsm;
    synapse_msgs_Safety safety;
    syn_sub_t sub_actuators, sub_battery_state,
        sub_fsm, sub_estimator_odometry, sub_safety;
    TinyFrame tf;
    volatile int client;
    int error_count;
    uint8_t rx1_buf[RX_BUF_SIZE];
    int serv;
    struct sockaddr_in bind_addr;
    int counter;
} context_t;

static context_t g_ctx = {
    .node = { 0 },
    .actuators = synapse_msgs_Actuators_init_default,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .estimator_odometry = synapse_msgs_Odometry_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .sub_actuators = {},
    .sub_battery_state = {},
    .sub_fsm = {},
    .sub_estimator_odometry = {},
    .sub_safety = {},
    .tf = {},
    .client = -1,
    .error_count = 0,
};

#define TOPIC_LISTENER(CHANNEL, CLASS)                                           \
    static TF_Result CHANNEL##_listener(TinyFrame* tf, TF_Msg* frame)            \
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

#define TOPIC_PUBLISHER(DATA, CLASS, TOPIC)                                   \
    {                                                                         \
        TF_Msg msg;                                                           \
        TF_ClearMsg(&msg);                                                    \
        uint8_t buf[CLASS##_size];                                            \
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));  \
        int status = pb_encode(&stream, CLASS##_fields, DATA);                \
        if (status) {                                                         \
            msg.type = TOPIC;                                                 \
            msg.data = buf;                                                   \
            msg.len = stream.bytes_written;                                   \
            TF_Send(&g_ctx.tf, &msg);                                         \
        } else {                                                              \
            printf("%s encoding failed: %s\n", #DATA, PB_GET_ERROR(&stream)); \
        }                                                                     \
    }

static void write_ethernet(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    if (g_ctx.client < 0) {
        return;
    }

    int out_len;
    const char* p;
    p = buf;
    do {
        out_len = zsock_send(g_ctx.client, p, len, 0);
        if (out_len < 0) {
            LOG_DBG("send: %d\n", errno);
            if (g_ctx.error_count++ > 10) {
                // trigger reconnect
                g_ctx.client = -1;
            }
            return;
        } else {
            // reset error count
            g_ctx.error_count = 0;
        }
        p += out_len;
        len -= out_len;
    } while (len);
}

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    LOG_WRN("unhandled tinyframe type: %4d", msg->type);
    // dumpFrameInfo(msg);
    return TF_STAY;
}

// ROS -> Cerebri
// TOPIC_LISTENER(actuators, synapse_msgs_Actuators)
// TOPIC_LISTENER(altimeter, synapse_msgs_Altimeter)
// TOPIC_LISTENER(external_odometry, synapse_msgs_Odometry)
TOPIC_LISTENER(bezier_trajectory, synapse_msgs_BezierTrajectory)
TOPIC_LISTENER(cmd_vel, synapse_msgs_Twist)
TOPIC_LISTENER(joy, synapse_msgs_Joy)
TOPIC_LISTENER(led_array, synapse_msgs_LEDArray)
TOPIC_LISTENER(clock_offset, synapse_msgs_Time)

#ifdef CONFIG_CEREBRI_DREAM_HIL
TOPIC_LISTENER(battery_state, synapse_msgs_BatteryState)
TOPIC_LISTENER(imu, synapse_msgs_Imu)
TOPIC_LISTENER(magnetic_field, synapse_msgs_MagneticField)
TOPIC_LISTENER(nav_sat_fix, synapse_msgs_NavSatFix)
TOPIC_LISTENER(wheel_odometry, synapse_msgs_WheelOdometry)
#endif

void listener_synapse_ethernet_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}

ZBUS_LISTENER_DEFINE(listener_synapse_ethernet, listener_synapse_ethernet_callback);
ZBUS_CHAN_ADD_OBS(chan_actuators, listener_synapse_ethernet, 1);
ZBUS_CHAN_ADD_OBS(chan_estimator_odometry, listener_synapse_ethernet, 1);
ZBUS_CHAN_ADD_OBS(chan_fsm, listener_synapse_ethernet, 1);
ZBUS_CHAN_ADD_OBS(chan_safety, listener_synapse_ethernet, 1);
ZBUS_CHAN_ADD_OBS(chan_battery_state, listener_synapse_ethernet, 1);

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

static void synapse_ethernet_init(context_t* ctx)
{
    syn_node_init(&ctx->node, "synapse_ethernet");
    syn_node_add_sub(&ctx->node, &ctx->sub_actuators, &ctx->actuators, &chan_actuators, 1);
    syn_node_add_sub(&ctx->node, &ctx->sub_battery_state, &ctx->battery_state, &chan_battery_state, 1);
    syn_node_add_sub(&ctx->node, &ctx->sub_estimator_odometry, &ctx->estimator_odometry, &chan_estimator_odometry, 10);
    syn_node_add_sub(&ctx->node, &ctx->sub_fsm, &ctx->fsm, &chan_fsm, 1);
    syn_node_add_sub(&ctx->node, &ctx->sub_safety, &ctx->safety, &chan_safety, 1);
}

static void send_uptime(context_t* ctx)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_Time_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int64_t ticks = k_uptime_ticks();
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    synapse_msgs_Time message;
    message.sec = sec;
    message.nanosec = nanosec;
    int status = pb_encode(&stream, synapse_msgs_Time_fields, &message);
    if (status) {
        msg.type = SYNAPSE_UPTIME_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&g_ctx.tf, &msg);
    } else {
        printf("uptime encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
}

static void ethernet_entry_point(context_t* ctx)
{
    synapse_ethernet_init(ctx);

    TF_InitStatic(&ctx->tf, TF_MASTER, write_ethernet);

    ctx->serv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    set_blocking_enabled(ctx->serv, true);

    if (ctx->serv < 0) {
        LOG_ERR("socket: %d", errno);
        exit(1);
    }

    ctx->bind_addr.sin_family = AF_INET;
    ctx->bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ctx->bind_addr.sin_port = htons(BIND_PORT);

    if (zsock_bind(ctx->serv, (struct sockaddr*)&ctx->bind_addr, sizeof(ctx->bind_addr)) < 0) {
        LOG_ERR("bind: %d", errno);
        exit(1);
    }

    if (zsock_listen(ctx->serv, 5) < 0) {
        LOG_ERR("listen: %d", errno);
        exit(1);
    }

    // ROS -> Cerebri
    TF_AddGenericListener(&ctx->tf, genericListener);
    // TF_AddTypeListener(&ctx->tf, SYNAPSE_ACTUATORS_TOPIC, actuators_listener);
    // TF_AddTypeListener(&ctx->tf, SYNAPSE_ALTIMETER_TOPIC, altimeter_listener);
    // TF_AddTypeListener(&ctx->tf, SYNAPSE_ODOMETRY_TOPIC, external_odometry_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_BEZIER_TRAJECTORY_TOPIC, bezier_trajectory_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_CMD_VEL_TOPIC, cmd_vel_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_JOY_TOPIC, joy_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_LED_ARRAY_TOPIC, led_array_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_CLOCK_OFFSET_TOPIC, clock_offset_listener);

#ifdef CONFIG_CEREBRI_DREAM_HIL
    TF_AddTypeListener(&ctx->tf, SYNAPSE_BATTERY_STATE_TOPIC, battery_state_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_IMU_TOPIC, imu_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_MAGNETIC_FIELD_TOPIC, magnetic_field_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_NAV_SAT_FIX_TOPIC, nav_sat_fix_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_WHEEL_ODOMETRY_TOPIC, wheel_odometry_listener);
#endif

    while (1) {
        LOG_INF("socket waiting for connection on port: %d", BIND_PORT);
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        g_ctx.client = zsock_accept(ctx->serv, (struct sockaddr*)&client_addr,
            &client_addr_len);

        if (g_ctx.client < 0) {
            continue;
        }

        zsock_inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        LOG_INF("connection #%d from %s", ctx->counter++, addr_str);

        while (1) {
            k_timeout_t timeout = K_MSEC(1);

            if (syn_sub_poll(&ctx->sub_battery_state, timeout) == 0) {
                TOPIC_PUBLISHER(&ctx->battery_state, synapse_msgs_BatteryState, SYNAPSE_BATTERY_STATE_TOPIC);
            }

            if (syn_sub_poll(&ctx->sub_fsm, timeout) == 0) {
                TOPIC_PUBLISHER(&ctx->fsm, synapse_msgs_Fsm, SYNAPSE_FSM_TOPIC);
            }

            if (syn_sub_poll(&ctx->sub_safety, timeout) == 0) {
                TOPIC_PUBLISHER(&ctx->safety, synapse_msgs_Safety, SYNAPSE_SAFETY_TOPIC);
            }

            if (syn_sub_poll(&ctx->sub_estimator_odometry, timeout) == 0) {
                TOPIC_PUBLISHER(&ctx->estimator_odometry, synapse_msgs_Odometry, SYNAPSE_ODOMETRY_TOPIC);
            }

            send_uptime(ctx);
            if (g_ctx.client < 0) {
                LOG_WRN("no client, triggering reconnect");
                break;
            }
            int len = zsock_recv(g_ctx.client, ctx->rx1_buf, sizeof(ctx->rx1_buf), 0);
            if (len < 0) {
                continue;
            }
            TF_Accept(&ctx->tf, ctx->rx1_buf, len);
            TF_Tick(&ctx->tf);
        }
    }
}

K_THREAD_DEFINE(synapse_ethernet, MY_STACK_SIZE, ethernet_entry_point,
    &g_ctx, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
