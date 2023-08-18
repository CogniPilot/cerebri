/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <synapse/zbus/channels.h>
#include <synapse/zbus/common.h>
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
typedef struct ctx_ {
    synapse_msgs_WheelOdometry sub_wheel_odometry;
    synapse_msgs_Actuators sub_actuators;
    synapse_msgs_Odometry pub_odometry;
    volatile bool actuators_updated;
    bool initialized;
    double x[3];
} ctx_t;

struct k_poll_signal signal;

// private initialization
static ctx_t ctx = {
    .sub_wheel_odometry = synapse_msgs_WheelOdometry_init_default,
    .sub_actuators = synapse_msgs_Actuators_init_default,
    .pub_odometry = synapse_msgs_Odometry_init_default,
    .actuators_updated = false,
    .initialized = false,
    .x = { 0 },
};

void listener_estimate_rover2d_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_wheel_odometry) {
        ctx.sub_wheel_odometry = *(synapse_msgs_WheelOdometry*)(chan->message);
        k_poll_signal_raise(&signal, 0x1337);
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

void handle_update(double* x1)
{
    bool x1_finite = all_finite(x1, sizeof(ctx.x));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (x1_finite) {
        memcpy(ctx.x, x1, sizeof(ctx.x));
    }
}

ZBUS_LISTENER_DEFINE_DISABLED(listener_estimate_rover2d, listener_estimate_rover2d_callback);

static void estimate_rover2d_entry_point(void* p1, void* p2, void* p3)
{
    // LOG_DBG("started");

    // variables
    int32_t seq = 0;
    double rotation_last = 0;
    k_poll_signal_init(&signal);

    struct k_poll_event events[1] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
            K_POLL_MODE_NOTIFY_ONLY,
            &signal),
    };

    // enable the zbus listener
    int ret = zbus_obs_set_enable(&listener_estimate_rover2d, true);
    if (ret != 0) {
        LOG_ERR("could not enable observer: %d", ret);
    }

    // estimator state and sqrt covariance
    while (true) {
        int ret = k_poll(events, 1, K_MSEC(1000));
        events[0].signal->signaled = 0;
        events[0].state = K_POLL_STATE_NOT_READY;
        if (ret != 0) {
            LOG_ERR("not receiving wheel odometry");
            k_usleep(1000);
            continue;
        }

        // get data
        double rotation = ctx.sub_wheel_odometry.rotation;

        // wait for valid steering angle and odometry to initialize
        if (!ctx.initialized) {
            if (ctx.actuators_updated) {
                rotation_last = ctx.sub_wheel_odometry.rotation;
                ctx.initialized = true;
                LOG_DBG("initialized: %d", ctx.initialized);
            } else {
                // wait for actuators update
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
            double delta = ctx.sub_actuators.position[0];
            // negative sign due to current gearing, should be in driver
            double u = (rotation - rotation_last) * D / 2;
            rotation_last = rotation;

            // memory
            static casadi_int iw[predict_SZ_IW];
            static casadi_real w[predict_SZ_W];

            // input
            LOG_DBG("delta: %10.4f u: %10.4f l: %10.4f", delta, u, l);
            const double* args[predict_SZ_ARG] = { ctx.x, &delta, &u, &l };

            // output
            double x1[3];
            double* res[predict_SZ_RES] = { x1 };

            // evaluate
            predict(args, res, iw, w, 0);

            // update x, W
            handle_update(x1);
        }

        // publish odometry
        {
            ctx.pub_odometry.has_header = true;
            const char frame_id[] = "odom";
            const char child_frame_id[] = "base_link";
            strncpy(
                ctx.pub_odometry.header.frame_id,
                frame_id, sizeof(ctx.pub_odometry.header.frame_id) - 1);
            strncpy(
                ctx.pub_odometry.child_frame_id,
                child_frame_id, sizeof(ctx.pub_odometry.child_frame_id) - 1);

            ctx.pub_odometry.has_header = true;
            int64_t uptime_ticks = k_uptime_ticks();
            int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
            int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

            ctx.pub_odometry.header.seq = seq++;
            ctx.pub_odometry.header.has_stamp = true;
            ctx.pub_odometry.header.stamp.sec = sec;
            ctx.pub_odometry.header.stamp.nanosec = nanosec;

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
        ctx.actuators_updated = false;
    }
}

K_THREAD_DEFINE(estimate_rover2d, MY_STACK_SIZE, estimate_rover2d_entry_point, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
/* vi: ts=4 sw=4 et */
