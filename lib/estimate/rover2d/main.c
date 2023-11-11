/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define casadi_real double
#define casadi_int int64_t

#include "casadi/rover2d.h"

LOG_MODULE_REGISTER(estimate_rover2d, CONFIG_CEREBRI_ESTIMATE_ROVER2D_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// private context
typedef struct _context {
    syn_node_t node;
    synapse_msgs_WheelOdometry wheel_odometry;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Odometry odometry;
    syn_sub_t sub_wheel_odometry, sub_actuators;
    syn_pub_t pub_odometry;
    double x[3];
} context;

// private initialization
static context g_ctx = {
    .node = { 0 },
    .wheel_odometry = synapse_msgs_WheelOdometry_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .odometry = {
        .child_frame_id = "base_link",
        .has_header = true,
        .header.frame_id = "odom",
        .has_pose = true,
        .pose.has_pose = true,
        .pose.pose.has_position = true,
        .pose.pose.has_orientation = true,
    },
    .sub_wheel_odometry = { 0 },
    .sub_actuators = { 0 },
    .pub_odometry = { 0 },
    .x = { 0 },
};

static void estimate_rover2d_init(context* ctx)
{
    syn_node_init(&ctx->node, "rover2d");
    syn_node_add_sub(&ctx->node, &ctx->sub_actuators, &ctx->actuators, &chan_actuators);
    syn_node_add_sub(&ctx->node, &ctx->sub_wheel_odometry,
        &ctx->wheel_odometry, &chan_wheel_odometry);
    syn_node_add_pub(&ctx->node, &ctx->pub_odometry, &ctx->odometry, &chan_estimator_odometry);
}

static void log_x(double* x)
{
    LOG_DBG("%10.4f %10.4f %10.4f",
        x[0], x[1], x[2]);
}

static bool all_finite(double* src, size_t n)
{
    for (int i = 0; i < n; i++) {
        if (!isfinite(src[i])) {
            return false;
        }
    }
    return true;
}

static void handle_update(context* ctx, double* x1)
{
    bool x1_finite = all_finite(x1, sizeof(ctx->x));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (x1_finite) {
        memcpy(ctx->x, x1, sizeof(ctx->x));
    }
}

static void listener_estimate_rover2d_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}
ZBUS_LISTENER_DEFINE_WITH_ENABLE(listener_estimate_rover2d, listener_estimate_rover2d_callback, false);
ZBUS_CHAN_ADD_OBS(chan_actuators, listener_estimate_rover2d, 1);
ZBUS_CHAN_ADD_OBS(chan_wheel_odometry, listener_estimate_rover2d, 1);

static void estimate_rover2d_run(context* ctx)
{
    // LOG_DBG("started");
    estimate_rover2d_init(ctx);

    // variables
    int32_t seq = 0;
    double rotation_last = 0;

    // enable the zbus listener
    int ret = zbus_obs_set_enable(&listener_estimate_rover2d, true);
    if (ret != 0) {
        LOG_ERR("could not enable observer: %d", ret);
    }

    // wait for actuators
    LOG_DBG("waiting for actuators");
    while (true) {
        int ret = syn_sub_poll(&ctx->sub_actuators, K_MSEC(100));
        if (ret == 0) {
            break;
        }
    }
    LOG_DBG("received actuators");

    // estimator state
    while (true) {
        // poll for wheel odometry
        int ret = syn_sub_poll(&ctx->sub_wheel_odometry, K_MSEC(1000));
        if (ret != 0) {
            LOG_DBG("not receiving wheel odometry");
            continue;
        }

        syn_node_lock_all(&ctx->node, K_MSEC(1));

        // get data
        double rotation = ctx->wheel_odometry.rotation;
        double delta = ctx->actuators.position[0];

#ifdef CONFIG_DREAM_SIL
        static const double l = 0.3; // adjusted distance to account for wheel slip in sim
#else
        static const double l = 0.2255; // distance between axles
#endif
        static const double D = 0.073; // wheel diameter

        // negative sign due to current gearing, should be in driver
        double u = (rotation - rotation_last) * D / 2;
        rotation_last = rotation;

        /* predict:(x0[3],delta,u,l)->(x1[3]) */
        {
            // LOG_DBG("predict");

            // memory
            static casadi_int iw[predict_SZ_IW];
            static casadi_real w[predict_SZ_W];

            // input
            LOG_DBG("delta: %10.4f u: %10.4f l: %10.4f", delta, u, l);
            const double* args[predict_SZ_ARG] = { ctx->x, &delta, &u, &l };

            // output
            double x1[3];
            double* res[predict_SZ_RES] = { x1 };

            // evaluate
            predict(args, res, iw, w, 0);

            // update x, W
            handle_update(ctx, x1);
        }

        // update odometry
        {
            stamp_header(&ctx->odometry.header, k_uptime_ticks());
            ctx->odometry.header.seq = seq++;

            double theta = ctx->x[2];
            ctx->odometry.pose.pose.position.x = ctx->x[0];
            ctx->odometry.pose.pose.position.y = ctx->x[1];
            ctx->odometry.pose.pose.position.z = 0;
            ctx->odometry.pose.pose.orientation.x = 0;
            ctx->odometry.pose.pose.orientation.y = 0;
            ctx->odometry.pose.pose.orientation.z = sin(theta / 2);
            ctx->odometry.pose.pose.orientation.w = cos(theta / 2);
            ctx->odometry.twist.twist.angular.z = u * tan(delta) / l;
            ctx->odometry.twist.twist.linear.x = u;
        }
        syn_node_publish_all(&ctx->node, K_MSEC(1));
        syn_node_unlock_all(&ctx->node);
        log_x(ctx->x);
    }
}

K_THREAD_DEFINE(estimate_rover2d, MY_STACK_SIZE, estimate_rover2d_run,
    &g_ctx, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
