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

#include "casadi/gen/elm4.h"

#include <cerebri/core/casadi.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(elm4_position, CONFIG_CEREBRI_ELM4_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_BezierTrajectory bezier_trajectory;
    synapse_msgs_Time clock_offset;
    synapse_msgs_Odometry pose;
    synapse_msgs_Twist cmd_vel;
    struct zros_sub sub_status, sub_clock_offset, sub_pose, sub_bezier_trajectory;
    struct zros_pub pub_cmd_vel;
    const double wheel_base;
    const double gain_along_track;
    const double gain_cross_track;
    const double gain_heading;
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
    .wheel_base = CONFIG_CEREBRI_ELM4_WHEEL_BASE_MM / 1000.0,
    .gain_along_track = CONFIG_CEREBRI_ELM4_GAIN_ALONG_TRACK / 1000.0,
    .gain_cross_track = CONFIG_CEREBRI_ELM4_GAIN_CROSS_TRACK / 1000.0,
    .gain_heading = CONFIG_CEREBRI_ELM4_GAIN_HEADING / 1000.0,
};

static void init(context* ctx)
{
    zros_node_init(&ctx->node, "elm4_position");
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

    double T = (time_stop_nsec - time_start_nsec) * 1e-9;
    double t = (time_nsec - time_start_nsec) * 1e-9;
    double x, y, psi, V, omega = 0;
    double e[3] = {}; // e_x, e_y, e_theta

    double PX[6], PY[6];
    for (int i = 0; i < 6; i++) {
        PX[i] = ctx->bezier_trajectory.curves[curve_index].x[i];
        PY[i] = ctx->bezier_trajectory.curves[curve_index].y[i];
    }

    /* bezier6_rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,omega) */
    {
        CASADI_FUNC_ARGS(bezier6_rover);
        args[0] = &t;
        args[1] = &T;
        args[2] = PX;
        args[3] = PY;
        res[0] = &x;
        res[1] = &y;
        res[2] = &psi;
        res[3] = &V;
        res[4] = &omega;
        CASADI_FUNC_CALL(bezier6_rover);
    }

    /* se2_error:(p[3],r[3])->(error[3]) */
    {
        double p[3], r[3];

        // vehicle position
        p[0] = ctx->pose.pose.pose.position.x;
        p[1] = ctx->pose.pose.pose.position.y;
        p[2] = 2 * atan2(ctx->pose.pose.pose.orientation.z, ctx->pose.pose.pose.orientation.w);

        // reference position
        r[0] = x;
        r[1] = y;
        r[2] = psi;

        // call function
        CASADI_FUNC_ARGS(se2_error);
        args[0] = p;
        args[1] = r;
        res[0] = e;
        CASADI_FUNC_CALL(se2_error);
    }

    // compute twist
    ctx->cmd_vel.linear.x = V + ctx->gain_along_track * e[0];
    ctx->cmd_vel.angular.z = omega + ctx->gain_cross_track * e[1] + ctx->gain_heading * e[2];
}

static void elm4_position_entry_point(void* p0, void* p1, void* p2)
{
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

K_THREAD_DEFINE(elm4_position, MY_STACK_SIZE,
    elm4_position_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
