/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>

#include "casadi/gen/rdd2.h"

#include <cerebri/core/casadi.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(rdd2_position, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_BezierTrajectory bezier_trajectory;
    synapse_msgs_Time clock_offset;
    synapse_msgs_Odometry pose;
    synapse_msgs_Twist cmd_vel;
    struct zros_sub sub_status, sub_clock_offset, sub_pose, sub_bezier_trajectory;
    struct zros_pub pub_cmd_vel;
} context;

static context g_ctx = {
    .status = synapse_msgs_Status_init_default,
    .bezier_trajectory = synapse_msgs_BezierTrajectory_init_default,
    .clock_offset = synapse_msgs_Time_init_default,
    .pose = synapse_msgs_Odometry_init_default,
    .cmd_vel = {
        .has_angular = true,
        .has_linear = true,
        .linear = synapse_msgs_Vector3_init_default,
        .angular = synapse_msgs_Vector3_init_default,
    },
    .sub_status = {},
    .sub_clock_offset = {},
    .sub_pose = {},
    .sub_bezier_trajectory = {},
    .pub_cmd_vel = {},
};

static void init(context* ctx)
{
    zros_node_init(&ctx->node, "rdd2_position");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_clock_offset, &ctx->node, &topic_clock_offset, &ctx->clock_offset, 10);
    zros_sub_init(&ctx->sub_pose, &ctx->node, &topic_estimator_odometry, &ctx->pose, 10);
    zros_sub_init(&ctx->sub_bezier_trajectory, &ctx->node, &topic_bezier_trajectory, &ctx->bezier_trajectory, 10);
    zros_pub_init(&ctx->pub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel);
}

static void stop(context* ctx)
{
    ctx->cmd_vel.linear.x = 0;
    ctx->cmd_vel.angular.z = 0;
}

// computes thrust/steering in auto mode
static void auto_mode(context* ctx)
{
    // goal -> given position goal, find cmd_vel
    uint64_t time_start_nsec = ctx->bezier_trajectory.time_start;
    uint64_t time_stop_nsec = time_start_nsec;

    // get current time
    uint64_t time_nsec = k_uptime_get() * 1e6 + ctx->clock_offset.sec * 1e9 + ctx->clock_offset.nanosec;

    if (time_nsec < time_start_nsec) {
        LOG_DBG("time current: %" PRIu64
                " ns < time start: %" PRIu64
                "  ns, time out of range of trajectory\n",
            time_nsec, time_start_nsec);
        stop(ctx);
        return;
    }

    // find current trajectory index, time_start, and time_stop
    int curve_index = 0;
    while (true) {

        // check if time handled by current trajectory
        if (time_nsec < ctx->bezier_trajectory.curves[curve_index].time_stop) {
            time_stop_nsec = ctx->bezier_trajectory.curves[curve_index].time_stop;
            if (curve_index > 0) {
                time_start_nsec = ctx->bezier_trajectory.curves[curve_index - 1].time_stop;
            }
            break;
        }

        // next index
        curve_index++;

        // check if index exceeds bounds
        if (curve_index >= ctx->bezier_trajectory.curves_count) {
            // LOG_ERR("curve index exceeds bounds");
            stop(ctx);
            return;
        }
    }
}

static void rdd2_position_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_pose),
    };

    while (true) {

        // LOG_DBG("polling on pose");
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("pos not receiving  pose");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_bezier_trajectory)) {
            zros_sub_update(&ctx->sub_bezier_trajectory);
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_pose)) {
            zros_sub_update(&ctx->sub_pose);
        }

        if (zros_sub_update_available(&ctx->sub_clock_offset)) {
            zros_sub_update(&ctx->sub_clock_offset);
        }

        if (ctx->status.mode != synapse_msgs_Status_Mode_MODE_AUTO) {
            // LOG_DBG("not auto mode");
            continue;
        }

        auto_mode(ctx);
        zros_pub_update(&ctx->pub_cmd_vel);
    }
}

K_THREAD_DEFINE(rdd2_position, MY_STACK_SIZE,
    rdd2_position_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
