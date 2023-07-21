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

#include "casadi/casadi_mrp.h"

LOG_MODULE_REGISTER(estimate_attitude, CONFIG_ESTIMATE_ATTITUDE_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

// private context
typedef struct ctx_ {
    synapse_msgs_Imu sub_imu;
    synapse_msgs_MagneticField sub_magnetic_field;
    synapse_msgs_Odometry pub_odometry;
    bool imu_updated;
    bool mag_updated;
    double x[7];
    double W[21];
} ctx_t;

// private initialization
static ctx_t ctx = {
    .sub_imu = synapse_msgs_Imu_init_default,
    .sub_magnetic_field = synapse_msgs_MagneticField_init_default,
    .pub_odometry = synapse_msgs_Odometry_init_default,
    .imu_updated = false,
    .mag_updated = false,
    .x = { 0 },
    .W = { 0 }
};

void listener_estimate_attitude_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_imu) {
        ctx.sub_imu = *(synapse_msgs_Imu*)(chan->message);
        ctx.imu_updated = true;
        // LOG_DBG("imu updated");
    } else if (chan == &chan_out_magnetic_field) {
        ctx.sub_magnetic_field = *(synapse_msgs_MagneticField*)(chan->message);
        ctx.mag_updated = true;
        // LOG_DBG("mag updated");
    }
}

void log_x(double* x)
{
    LOG_DBG("%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f",
        x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
}

void log_W(double* W)
{
    LOG_DBG("%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f",
        W[0], W[1], W[2], W[3], W[4], W[5], W[6]);
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
    bool W1_finite = all_finite(W1, sizeof(ctx.W));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (!W1_finite) {
        LOG_WRN("W1 update not finite");
    }

    if (x1_finite && W1_finite) {
        memcpy(ctx.x, x1, sizeof(ctx.x));
        memcpy(ctx.W, W1, sizeof(ctx.W));
    }
}

ZBUS_LISTENER_DEFINE(listener_estimate_attitude, listener_estimate_attitude_callback);

static void estimate_attitude_entry_point(void* p1, void* p2, void* p3)
{
    // variables
    double dt = 1.0 / 1.0;
    bool initialized = false;

    // parameters
    const double decl = 0;
    const double std_gyro = 1.0;
    const double sn_gyro_rw = 0.01;
    const double beta_mag_c = 6.6;
    const double std_mag = 25.0;
    const double g = 9.8;
    const double std_accel = 35.0;
    const double std_accel_omega = 0.1;
    const double beta_accel_c = 9.2;

    double y_accel[3], y_gyro[3], y_mag[3];

    // estimator state and sqrt covariance
    while (true) {
        // sleep to set rate
        k_usleep(dt * 1e6);

        // continue if no new data
        if (!ctx.mag_updated && !ctx.imu_updated)
            continue;

        // get data
        if (ctx.imu_updated) {
            y_accel[0] = ctx.sub_imu.linear_acceleration.x;
            y_accel[1] = ctx.sub_imu.linear_acceleration.y;
            y_accel[2] = ctx.sub_imu.linear_acceleration.z;
            LOG_DBG("accel: %10.4f %10.4f %10.4f", y_accel[0], y_accel[1], y_accel[2]);

            y_gyro[0] = ctx.sub_imu.angular_velocity.x;
            y_gyro[1] = ctx.sub_imu.angular_velocity.y;
            y_gyro[2] = ctx.sub_imu.angular_velocity.z;
            LOG_DBG("gyro: %10.4f %10.4f %10.4f", y_gyro[0], y_gyro[1], y_gyro[2]);
        }

        if (ctx.mag_updated) {
            y_mag[0] = ctx.sub_magnetic_field.magnetic_field.x;
            y_mag[1] = ctx.sub_magnetic_field.magnetic_field.y;
            y_mag[2] = ctx.sub_magnetic_field.magnetic_field.z;
            LOG_DBG("mag: %10.4f %10.4f %10.4f", y_mag[0], y_mag[1], y_mag[2]);
        }

        /* init:(g_b[3],B_b[3],decl)->(x0[7],error_code) */
        if (!initialized && ctx.mag_updated && ctx.imu_updated) {
            LOG_DBG("init");

            // memory
            static casadi_int iw[init_SZ_IW];
            static casadi_real w[init_SZ_W];

            // variables
            double x0[7] = { 0 };
            double init_ret = 0;

            // input
            const double* args[] = { y_accel, y_mag, &decl };

            // output
            double* res[] = { x0, &init_ret };

            // evaluate
            init(args, res, iw, w, 0);
            log_x(x0);

            // handle
            int init_flag = (int)init_ret;
            if (init_flag == 0) {
                initialized = true;

                // update x
                if (all_finite(x0, sizeof(x0))) {
                    memcpy(ctx.x, x0, sizeof(ctx.x));
                }
                LOG_DBG("initialized");
                log_x(ctx.x);
                log_W(ctx.W);
            } else {
                LOG_DBG("initialization failed: %d, %10.4f", init_flag, init_ret);
            }

            ctx.mag_updated = false;
            ctx.imu_updated = false;
            continue;
        }

        // predict:(t,x[7],W[6x6,21nz],omega_m[3],std_gyro,sn_gyro_rw,dt)->
        //     (x1[7],W1[6x6,21nz]) */
        if (ctx.imu_updated) {
            LOG_DBG("predict");

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
        }

        // correct_mag:(x[7],W[6x6,21nz],y_b[3],decl,std_mag,beta_mag_c)->
        //     (x_mag[7],W_mag[6x6,21nz],beta_mag,r_mag,r_std_mag,error_code)
        if (ctx.mag_updated) {
            LOG_DBG("correct mag");

            // memory
            static casadi_int iw[correct_mag_SZ_IW];
            static casadi_real w[correct_mag_SZ_W];

            // variables
            double beta_mag = 0;
            double r_mag = 0;
            double r_std_mag = 0;
            double ret_mag = 0;

            // input
            const double* args[] = {
                ctx.x, ctx.W, y_mag, &decl, &std_mag, &beta_mag_c
            };

            // output
            double x1[7];
            double W1[21];
            double* res[] = { x1, W1, &beta_mag, &r_mag, &r_std_mag, &ret_mag };

            // evaluate
            correct_mag(args, res, iw, w, 0);

            // update x, W
            handle_update(x1, W1);
        }

        // correct_accel:(x[7],W[6x6,21nz],y_b[3],g,omega_b[3],
        //         std_accel,std_accel_omega,beta_accel_c)->
        //     (x_accel[7],W_accel[6x6,21nz],
        //         beta_accel,r_accel[2],r_std_accel[2],error_code)
        if (ctx.imu_updated) {
            LOG_DBG("correct accel");

            // memory
            static casadi_int iw[correct_accel_SZ_IW];
            static casadi_real w[correct_accel_SZ_W];

            // variables
            double beta_accel = 0;
            double r_accel[2] = { 0 };
            double r_std_accel[2] = { 0 };
            double ret_accel = 0;

            // input
            const double* args[] = {
                ctx.x, ctx.W, y_accel, &g, y_gyro, &std_accel, &std_accel_omega, &beta_accel_c
            };

            // output
            double x1[7] = {};
            double W1[21] = {};
            double* res[] = {
                x1, W1, &beta_accel, r_accel, r_std_accel, &ret_accel
            };

            // evaluate
            correct_accel(args, res, iw, w, 0);

            // update x, W
            handle_update(ctx.x, ctx.W);
        }

        // publish xyz
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

            // TODO, publish quaternion from MRP
            ctx.pub_odometry.pose.pose.orientation.x = 0;
            ctx.pub_odometry.pose.pose.orientation.y = 0;
            ctx.pub_odometry.pose.pose.orientation.z = 0;
            ctx.pub_odometry.pose.pose.orientation.w = 1;
            zbus_chan_pub(&chan_out_odometry, &ctx.pub_odometry, K_NO_WAIT);
        }

        log_x(ctx.x);
        log_W(ctx.W);

        // set data as old now
        ctx.imu_updated = false;
        ctx.mag_updated = false;
    }
}

K_THREAD_DEFINE(estimate_attitude_thread, MY_STACK_SIZE,
    estimate_attitude_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
