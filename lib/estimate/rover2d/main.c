/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <synapse/zbus/common.h>
#include <synapse/zbus/syn_pub_sub.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define casadi_real double
#define casadi_int int64_t

#include "casadi/rover2d.h"

LOG_MODULE_REGISTER(estimate_rover2d, CONFIG_ESTIMATE_ROVER2D_LOG_LEVEL);

#define MY_STACK_SIZE 2130
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// private context
typedef struct _context {
    synapse_msgs_WheelOdometry wheel_odometry;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Odometry odometry;
    struct syn_sub sub_wheel_odometry, sub_actuators;
    struct syn_pub pub_odometry;
    bool initialized;
    double x[3];
} context;

// private initialization
static context g_ctx = {
    .wheel_odometry = synapse_msgs_WheelOdometry_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .odometry = synapse_msgs_Odometry_init_default,
    .sub_wheel_odometry = { 0 },
    .sub_actuators = { 0 },
    .pub_odometry = { 0 },
    .initialized = false,
    .x = { 0 },
};

static void init(context* ctx)
{
    syn_sub_init(&ctx->sub_actuators, &ctx->actuators, &chan_out_actuators);
    syn_sub_init(&ctx->sub_wheel_odometry, &ctx->wheel_odometry, &chan_out_wheel_odometry);
    syn_pub_init(&ctx->pub_odometry, &ctx->odometry, &chan_out_odometry);
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
    syn_sub_listen(&g_ctx.sub_actuators, chan, K_MSEC(1));
    syn_sub_listen(&g_ctx.sub_wheel_odometry, chan, K_MSEC(1));
}
ZBUS_LISTENER_DEFINE_WITH_ENABLE(listener_estimate_rover2d, listener_estimate_rover2d_callback, false);
ZBUS_CHAN_ADD_OBS(chan_out_actuators, listener_estimate_rover2d, 1);
ZBUS_CHAN_ADD_OBS(chan_out_wheel_odometry, listener_estimate_rover2d, 1);

static void run(context* ctx)
{
    // LOG_DBG("started");
    init(ctx);

    // variables
    int32_t seq = 0;
    double rotation_last = 0;

    // enable the zbus listener
    int ret = zbus_obs_set_enable(&listener_estimate_rover2d, true);
    if (ret != 0) {
        LOG_ERR("could not enable observer: %d", ret);
    }

    // estimator state and sqrt covariance
    while (true) {

        int ret = syn_sub_poll(&ctx->sub_wheel_odometry, K_MSEC(1000));
        if (ret != 0) {
            LOG_ERR("not receiving wheel odometry");
            continue;
        }

        // get data
        double rotation = ctx->wheel_odometry.rotation;

        // wait for valid steering angle and odometry to initialize
        if (!ctx->initialized) {
            if (ctx->actuators.header.seq != 0) {
                rotation_last = ctx->wheel_odometry.rotation;
                ctx->initialized = true;
                LOG_DBG("initialized: %d", ctx->initialized);
            } else {
                // wait for actuators update
                LOG_DBG("waiting for actuators update to initialize");
                continue;
            }
        }

        /* predict:(x0[3],delta,u,l)->(x1[3]) */
        {
            // LOG_DBG("predict");

#ifdef CONFIG_DREAM_SITL
            static const double l = 0.3; // adjusted distance to account for wheel slip in sim
#else
            static const double l = 0.2255; // distance between axles
#endif
            static const double D = 0.073; // wheel diameter
            double delta = ctx->actuators.position[0];
            // negative sign due to current gearing, should be in driver
            double u = (rotation - rotation_last) * D / 2;
            rotation_last = rotation;

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

        // publish odometry
        {
            ctx->odometry.has_header = true;
            const char frame_id[] = "odom";
            const char child_frame_id[] = "base_link";
            strncpy(
                ctx->odometry.header.frame_id,
                frame_id, sizeof(ctx->odometry.header.frame_id) - 1);
            strncpy(
                ctx->odometry.child_frame_id,
                child_frame_id, sizeof(ctx->odometry.child_frame_id) - 1);

            ctx->odometry.has_header = true;
            int64_t uptime_ticks = k_uptime_ticks();
            int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
            int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

            ctx->odometry.header.seq = seq++;
            ctx->odometry.header.has_stamp = true;
            ctx->odometry.header.stamp.sec = sec;
            ctx->odometry.header.stamp.nanosec = nanosec;

            ctx->odometry.has_pose = true;
            ctx->odometry.pose.has_pose = true;
            ctx->odometry.pose.pose.has_orientation = true;
            ctx->odometry.pose.pose.has_position = true;

            double theta = ctx->x[2];
            ctx->odometry.pose.pose.position.x = ctx->x[0];
            ctx->odometry.pose.pose.position.y = ctx->x[1];
            ctx->odometry.pose.pose.position.z = 0;
            ctx->odometry.pose.pose.orientation.x = 0;
            ctx->odometry.pose.pose.orientation.y = 0;
            ctx->odometry.pose.pose.orientation.z = sin(theta / 2);
            ctx->odometry.pose.pose.orientation.w = cos(theta / 2);

            syn_pub_publish(&ctx->pub_odometry, K_MSEC(1));
        }

        log_x(ctx->x);
    }
}

K_THREAD_DEFINE(estimate_rover2d, MY_STACK_SIZE, run,
    &g_ctx, NULL, NULL, MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
