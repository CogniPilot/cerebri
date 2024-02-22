/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include "proto/udp_rx.h"
#include <synapse_topic_list.h>

#include <pb_decode.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>
#include <synapse_tinyframe/utils.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 1

LOG_MODULE_REGISTER(eth_rx, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    struct udp_rx udp;
    TinyFrame tf;
    struct zros_sub sub_status;
    synapse_msgs_Status status;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .udp = {},
    .sub_status = {},
    .tf = {},
    .status = {},
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

#define TOPIC_LISTENER(CHANNEL, CLASS)                                            \
    static TF_Result CHANNEL##_listener(TinyFrame* tf, TF_Msg* frame)             \
    {                                                                             \
        CLASS msg = CLASS##_init_default;                                         \
        pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);    \
        int rc = pb_decode(&stream, CLASS##_fields, &msg);                        \
        if (rc) {                                                                 \
            zros_topic_publish(&topic_##CHANNEL, &msg);                           \
            LOG_DBG("%s decoding\n", #CHANNEL);                                   \
        } else {                                                                  \
            LOG_WRN("%s decoding failed: %s\n", #CHANNEL, PB_GET_ERROR(&stream)); \
        }                                                                         \
        return TF_STAY;                                                           \
    }

// topic listeners
TOPIC_LISTENER(bezier_trajectory, synapse_msgs_BezierTrajectory)
TOPIC_LISTENER(joy, synapse_msgs_Joy)
TOPIC_LISTENER(clock_offset, synapse_msgs_Time)
#ifdef CONFIG_CEREBRI_DREAM_HIL
TOPIC_LISTENER(battery_state, synapse_msgs_BatteryState)
TOPIC_LISTENER(imu, synapse_msgs_Imu)
TOPIC_LISTENER(magnetic_field, synapse_msgs_MagneticField)
TOPIC_LISTENER(nav_sat_fix, synapse_msgs_NavSatFix)
TOPIC_LISTENER(wheel_odometry, synapse_msgs_WheelOdometry)
#endif

static TF_Result genericListener(TinyFrame* tf, TF_Msg* msg)
{
    LOG_WRN("unhandled tinyframe type: %4d", msg->type);
    // dumpFrameInfo(msg);
    return TF_STAY;
}

static TF_Result cmd_vel_listener(TinyFrame* tf, TF_Msg* frame)
{
    struct context* ctx = tf->userdata;
    // don't publish cmd_vel if not in command vel mode
    if (ctx->status.mode != synapse_msgs_Status_Mode_MODE_CMD_VEL) {
        return TF_STAY;
    }
    synapse_msgs_Twist msg = synapse_msgs_Twist_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_Twist_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_cmd_vel, &msg);
        LOG_DBG("cmd_vel decoding\n");
    } else {
        LOG_WRN("cmd_vel decoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static int eth_rx_init(struct context* ctx)
{
    int ret = 0;
    // setup zros node
    zros_node_init(&ctx->node, "eth_rx");

    // setup udp connection
    ret = udp_rx_init(&ctx->udp);
    if (ret < 0) {
        LOG_ERR("udp rx init failed: %d", ret);
        return ret;
    }

    // initialize tinyframe
    ret = TF_InitStatic(&ctx->tf, TF_MASTER, NULL);
    if (ret < 0) {
        LOG_ERR("tf init failed: %d", ret);
        return ret;
    }
    ctx->tf.userdata = ctx;

    // add zros subscribers
    ret = zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);
    if (ret < 0) {
        LOG_ERR("sub status init failed: %d", ret);
        return ret;
    }

    // add tinyframe listeners
    ret = TF_AddGenericListener(&ctx->tf, genericListener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_BEZIER_TRAJECTORY_TOPIC, bezier_trajectory_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_CMD_VEL_TOPIC, cmd_vel_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_JOY_TOPIC, joy_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_CLOCK_OFFSET_TOPIC, clock_offset_listener);
    if (ret < 0)
        return ret;
#ifdef CONFIG_CEREBRI_DREAM_HIL
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_BATTERY_STATE_TOPIC, battery_state_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_IMU_TOPIC, imu_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_MAGNETIC_FIELD_TOPIC, magnetic_field_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_NAV_SAT_FIX_TOPIC, nav_sat_fix_listener);
    if (ret < 0)
        return ret;
    ret = TF_AddTypeListener(&ctx->tf, SYNAPSE_WHEEL_ODOMETRY_TOPIC, wheel_odometry_listener);
    if (ret < 0)
        return ret;
#endif

    atomic_set(&ctx->running, 1);
    return ret;
};

static int eth_rx_fini(struct context* ctx)
{
    int ret = 0;
    // close udp socket
    ret = udp_rx_fini(&ctx->udp);

    // close subscriptions
    zros_sub_fini(&ctx->sub_status);
    atomic_set(&ctx->running, 0);
    return ret;
};

static void eth_rx_run(void* p0, void* p1, void* p2)
{
    int ret = 0;
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    // constructor
    ret = eth_rx_init(ctx);
    if (ret < 0) {
        LOG_ERR("init failed: %d", ret);
        return;
    }

    LOG_INF("running");

    // while running
    while (atomic_get(&ctx->running)) {
        // poll sockets and receive data
        int received = udp_rx_receive(&ctx->udp);
        if (received < 0) {
            LOG_ERR("connection error: %d", errno);
        } else if (received > 0) {
            TF_Accept(&ctx->tf, ctx->udp.rx_buf, received);
        }

        // update status
        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        // tell tinyframe time has passed
        TF_Tick(&ctx->tf);
    }

    // deconstructor
    ret = eth_rx_fini(ctx);
    if (ret < 0) {
        LOG_ERR("fini failed: %d", ret);
    }
};

static int eth_rx_start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        eth_rx_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "eth_rx");
    k_thread_start(tid);
    return 0;
}

static int eth_rx_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct context* ctx = data;
    assert(argc == 1);

    if (strcmp(argv[0], "start") == 0) {
        if (atomic_get(&ctx->running)) {
            shell_print(sh, "already running");
        } else {
            eth_rx_start(ctx);
        }
    } else if (strcmp(argv[0], "stop") == 0) {
        if (atomic_get(&ctx->running)) {
            atomic_set(&ctx->running, 0);
        } else {
            shell_print(sh, "not running");
        }
    } else if (strcmp(argv[0], "status") == 0) {
        shell_print(sh, "running: %d", (int)atomic_get(&ctx->running));
    }
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_eth_rx, eth_rx_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(eth_rx, &sub_eth_rx, "eth_rx commands", NULL);

static int eth_rx_sys_init(void)
{
    return eth_rx_start(&g_ctx);
};

SYS_INIT(eth_rx_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
