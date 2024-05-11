/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>

#include "casadi/gen/rdd2.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(rdd2_estimate, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
    struct zros_node node;
    synapse_msgs_Odometry external_odometry;
    synapse_msgs_Imu imu;
    synapse_msgs_Odometry odometry;
    struct zros_sub sub_external_odometry, sub_imu;
    struct zros_pub pub_odometry;
    double x[3];
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

// private initialization
static struct context g_ctx = {
    .node = {},
    .external_odometry = synapse_msgs_Odometry_init_default,
    .imu = synapse_msgs_Imu_init_default,
    .odometry = {
        .child_frame_id = "base_link",
        .has_header = true,
        .header.frame_id = "odom",
        .has_pose = true,
        .pose.has_pose = true,
        .pose.pose.has_position = true,
        .pose.pose.has_orientation = true,
    },
    .sub_external_odometry = {},
    .sub_imu = {},
    .pub_odometry = {},
    .x = {},
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void rdd2_estimate_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_estimate");
    zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 100);
    zros_sub_init(&ctx->sub_external_odometry, &ctx->node, &topic_external_odometry,
        &ctx->external_odometry, 100);
    zros_pub_init(&ctx->pub_odometry, &ctx->node, &topic_estimator_odometry, &ctx->odometry);
    atomic_set(&ctx->running, 1);
}

static void rdd2_estimate_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_sub_fini(&ctx->sub_imu);
    zros_sub_fini(&ctx->sub_external_odometry);
    zros_pub_fini(&ctx->pub_odometry);
    atomic_set(&ctx->running, 0);
}

static void rdd2_estimate_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int rc = 0;

    // LOG_DBG("started");
    rdd2_estimate_init(ctx);

    // variables
    int32_t seq = 0;

    struct k_poll_event events[1] = {};

    // wait for imu
    LOG_DBG("waiting for imu");
    events[0] = *zros_sub_get_event(&ctx->sub_imu);
    rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
    if (rc != 0) {
        LOG_DBG("did not receive imu");
        return;
    }
    if (zros_sub_update_available(&ctx->sub_imu)) {
        zros_sub_update(&ctx->sub_imu);
    }

    // wait for external odometry
    LOG_DBG("waiting for external odometry");
    events[0] = *zros_sub_get_event(&ctx->sub_external_odometry);
    rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
    if (rc != 0) {
        LOG_DBG("did not receive external odometry");
        return;
    }
    if (zros_sub_update_available(&ctx->sub_external_odometry)) {
        zros_sub_update(&ctx->sub_external_odometry);
    }

    double dt = 0;
    int64_t ticks_last = k_uptime_ticks();

    // poll on imu
    events[0] = *zros_sub_get_event(&ctx->sub_imu);

    while (atomic_get(&ctx->running)) {

        // poll for imu
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("not receiving imu");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_imu)) {
            zros_sub_update(&ctx->sub_imu);
        }

        if (zros_sub_update_available(&ctx->sub_external_odometry)) {
            zros_sub_update(&ctx->sub_external_odometry);
        }

        // calculate dt
        int64_t ticks_now = k_uptime_ticks();
        dt = (float)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        ticks_last = ticks_now;
        if (dt < 0 || dt > 0.5) {
            LOG_WRN("imu update rate too low");
            continue;
        }

        // publish odometry
        {
            stamp_header(&ctx->odometry.header, k_uptime_ticks());
            ctx->odometry.header.seq = seq++;

            // state feedback for now
            ctx->odometry.pose.pose.position.x = ctx->external_odometry.pose.pose.position.x;
            ctx->odometry.pose.pose.position.y = ctx->external_odometry.pose.pose.position.y;
            ctx->odometry.pose.pose.position.z = ctx->external_odometry.pose.pose.position.z;
            ctx->odometry.pose.pose.orientation.x = ctx->external_odometry.pose.pose.orientation.x;
            ctx->odometry.pose.pose.orientation.y = ctx->external_odometry.pose.pose.orientation.y;
            ctx->odometry.pose.pose.orientation.z = ctx->external_odometry.pose.pose.orientation.z;
            ctx->odometry.pose.pose.orientation.w = ctx->external_odometry.pose.pose.orientation.w;

            // use gyro
            ctx->odometry.twist.twist.angular.x = ctx->imu.angular_velocity.x;
            ctx->odometry.twist.twist.angular.y = ctx->imu.angular_velocity.y;
            ctx->odometry.twist.twist.angular.z = ctx->imu.angular_velocity.z;

            // ctx->odometry.twist.twist.angular.x = ctx->external_odometry.twist.twist.angular.x;
            // ctx->odometry.twist.twist.angular.y = ctx->external_odometry.twist.twist.angular.y;
            // ctx->odometry.twist.twist.angular.z = ctx->external_odometry.twist.twist.angular.z;

            ctx->odometry.twist.twist.linear.x = ctx->external_odometry.twist.twist.linear.x;
            ctx->odometry.twist.twist.linear.y = ctx->external_odometry.twist.twist.linear.y;
            ctx->odometry.twist.twist.linear.z = ctx->external_odometry.twist.twist.linear.z;
            zros_pub_update(&ctx->pub_odometry);
        }
    }

    rdd2_estimate_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_estimate_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_estimate");
    k_thread_start(tid);
    return 0;
}

static int rdd2_estimate_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct context* ctx = data;
    assert(argc == 1);

    if (strcmp(argv[0], "start") == 0) {
        if (atomic_get(&ctx->running)) {
            shell_print(sh, "already running");
        } else {
            start(ctx);
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_estimate, rdd2_estimate_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_estimate, &sub_rdd2_estimate, "rdd2 estimate commands", NULL);

static int rdd2_estimate_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_estimate_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
