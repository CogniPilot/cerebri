/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>

#include "casadi/gen/rdd2.h"

LOG_MODULE_REGISTER(rdd2_estimate, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// private context
typedef struct _context {
    struct zros_node node;
    synapse_msgs_Odometry external_odometry;
    synapse_msgs_Imu imu;
    synapse_msgs_Odometry odometry;
    struct zros_sub sub_external_odometry, sub_imu;
    struct zros_pub pub_odometry;
    double x[3];
} context;

// private initialization
static context g_ctx = {
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
};

static void estimate_rover2d_init(context* ctx)
{
    zros_node_init(&ctx->node, "rdd2_estimate");
    zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 10);
    zros_sub_init(&ctx->sub_external_odometry, &ctx->node, &topic_external_odometry,
        &ctx->external_odometry, 10);
    zros_pub_init(&ctx->pub_odometry, &ctx->node, &topic_estimator_odometry, &ctx->odometry);
}

/*
static bool all_finite(double* src, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (!isfinite(src[i])) {
            return false;
        }
    }
    return true;
}
*/

/*
static void handle_update(context* ctx, double* x1)
{
    bool x1_finite = all_finite(x1, ARRAY_SIZE(ctx->x));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (x1_finite) {
        memcpy(ctx->x, x1, sizeof(ctx->x));
    }
}
*/

static void rdd2_estimate_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int rc = 0;

    // LOG_DBG("started");
    estimate_rover2d_init(ctx);

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
        LOG_DBG("did not receive wheel odometry");
        return;
    }
    if (zros_sub_update_available(&ctx->sub_external_odometry)) {
        zros_sub_update(&ctx->sub_external_odometry);
    }

    double dt = 0;
    int64_t ticks_last = k_uptime_ticks();

    // poll on imu
    events[0] = *zros_sub_get_event(&ctx->sub_imu);

    // estimator state
    while (true) {

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
            ctx->odometry.twist.twist.angular.x = ctx->external_odometry.twist.twist.angular.x;
            ctx->odometry.twist.twist.angular.y = ctx->external_odometry.twist.twist.angular.y;
            ctx->odometry.twist.twist.angular.z = ctx->external_odometry.twist.twist.angular.z;
            ctx->odometry.twist.twist.linear.x = ctx->external_odometry.twist.twist.linear.x;
            ctx->odometry.twist.twist.linear.y = ctx->external_odometry.twist.twist.linear.y;
            ctx->odometry.twist.twist.linear.z = ctx->external_odometry.twist.twist.linear.z;
            zros_pub_update(&ctx->pub_odometry);
        }
    }
}

K_THREAD_DEFINE(rdd2_estimate, MY_STACK_SIZE, rdd2_estimate_entry_point,
    &g_ctx, NULL, NULL, MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
