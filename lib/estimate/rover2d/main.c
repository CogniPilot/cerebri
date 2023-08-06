/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <synapse/zbus/channels.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define casadi_real double
#define casadi_int int64_t

#include "casadi/rover2d.h"

LOG_MODULE_REGISTER(estimate_rover2d, CONFIG_ESTIMATE_ROVER2D_LOG_LEVEL);

#define MY_STACK_SIZE 2130
#define MY_PRIORITY 4

// private context
typedef struct ctx_ {
    synapse_msgs_WheelOdometry sub_wheel_odometry;
    synapse_msgs_Actuators sub_actuators;
    synapse_msgs_Odometry pub_odometry;
    bool wheel_odometry_updated;
    bool actuators_updated;
    double x[3];
} ctx_t;

// private initialization
static ctx_t ctx = {
    .sub_wheel_odometry = synapse_msgs_WheelOdometry_init_default,
    .sub_actuators = synapse_msgs_Actuators_init_default,
    .pub_odometry = synapse_msgs_Odometry_init_default,
    .wheel_odometry_updated = false,
    .actuators_updated = false,
    .x = { 0 },
};

void listener_estimate_rover2d_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_wheel_odometry) {
        ctx.sub_wheel_odometry = *(synapse_msgs_WheelOdometry*)(chan->message);
        ctx.wheel_odometry_updated = true;
        // LOG_DBG("wheel odometry updated");
    } else if (chan == &chan_out_actuators) {
        ctx.sub_actuators = *(synapse_msgs_Actuators*)(chan->message);
        ctx.actuators_updated = true;
        // LOG_DBG("actuators updated");
    }
}

void log_x(double* x)
{
    LOG_DBG("%10.4f %10.4f %10.4f",
        x[0], x[1], x[2]);
}

bool all_finite(double* src, size_t n)
{
    for (int i = 0; i < n; i++) {
        if (!isfinite(src[i])) {
            return false;
        }
    }
    return true;
}

void handle_update(double* x1, double* W1)
{
    bool x1_finite = all_finite(x1, sizeof(ctx.x));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (x1_finite) {
        memcpy(ctx.x, x1, sizeof(ctx.x));
    }
}

ZBUS_LISTENER_DEFINE(listener_estimate_rover2d, listener_estimate_rover2d_callback);

static void estimate_rover2d_entry_point(void* p1, void* p2, void* p3)
{
    LOG_DBG("started");

    // variables
    double dt = 1.0 / 1.0;

    // estimator state and sqrt covariance
    while (true) {
        // sleep to set rate
        k_usleep(dt * 1e6);

        LOG_DBG("update");

        // continue if no new data
        if (!ctx.wheel_odometry_updated)
            continue;

        // get data
        if (ctx.wheel_odometry_updated) {
            double rotation = ctx.sub_wheel_odometry.rotation;
            LOG_DBG("rotation: %10.4f", rotation);
        }

        // predict:(t,x[7],W[6x6,21nz],omega_m[3],std_gyro,sn_gyro_rw,dt)->
        //     (x1[7],W1[6x6,21nz]) */
        if (ctx.wheel_odometry_updated) {
            LOG_DBG("predict");

            /*

            // memory
            static casadi_int iw[predict_SZ_IW];
            static casadi_real w[predict_SZ_W];

            // input
            const double* args[] = { &dt, ctx.x, ctx.W, y_gyro, &std_gyro, &sn_gyro_rw, &dt };

            // output
            double x1[7];
            double W1[21];
            double* res[] = { x1, W1 };

            // evaluate
            predict(args, res, iw, w, 0);

            // update x, W
            handle_update(x1, W1);
            */
        }

        // publish odometry
        {
            ctx.pub_odometry.has_header = true;
            const char frame_id[] = "map";
            const char child_frame_id[] = "base_link";
            strncpy(
                ctx.pub_odometry.header.frame_id,
                frame_id, sizeof(ctx.pub_odometry.header.frame_id) - 1);
            strncpy(
                ctx.pub_odometry.child_frame_id,
                child_frame_id, sizeof(ctx.pub_odometry.child_frame_id) - 1);
            ctx.pub_odometry.has_pose = true;
            ctx.pub_odometry.pose.has_pose = true;
            ctx.pub_odometry.pose.pose.has_orientation = true;
            ctx.pub_odometry.pose.pose.has_position = true;

            double theta = ctx.x[2];
            ctx.pub_odometry.pose.pose.position.x = ctx.x[0];
            ctx.pub_odometry.pose.pose.position.y = ctx.x[1];
            ctx.pub_odometry.pose.pose.position.z = 0;
            ctx.pub_odometry.pose.pose.orientation.x = 0;
            ctx.pub_odometry.pose.pose.orientation.y = 0;
            ctx.pub_odometry.pose.pose.orientation.z = sin(theta / 2);
            ctx.pub_odometry.pose.pose.orientation.w = cos(theta / 2);
            zbus_chan_pub(&chan_out_odometry, &ctx.pub_odometry, K_NO_WAIT);
        }

        log_x(ctx.x);

        // set data as old now
        ctx.wheel_odometry_updated = false;
        ctx.actuators_updated = false;
    }
}

K_THREAD_DEFINE(estimate_rover2d_thread, MY_STACK_SIZE, estimate_rover2d_entry_point, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
/* vi: ts=4 sw=4 et */
