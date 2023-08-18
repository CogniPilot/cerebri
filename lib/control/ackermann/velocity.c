/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/rover.h"
#include <math.h>

#include <zephyr/logging/log.h>

#include <synapse/zbus/common.h>
#include <synapse/zbus/syn_pub_sub.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_ackermann);

typedef struct _context {
    synapse_msgs_Twist cmd_vel;
    synapse_msgs_Joy joy;
    synapse_msgs_Fsm fsm;
    synapse_msgs_Actuators actuators;
    struct syn_sub sub_joy, sub_fsm, sub_cmd_vel;
    struct syn_pub pub_actuators;
    const double wheel_radius;
    const double wheel_base;
    const double max_turn_angle;
    const double max_velocity;
} context;

static context g_ctx = {
    .cmd_vel = synapse_msgs_Twist_init_default,
    .joy = synapse_msgs_Joy_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .sub_joy = { 0 },
    .sub_fsm = { 0 },
    .sub_cmd_vel = { 0 },
    .pub_actuators = { 0 },
    .wheel_radius = CONFIG_CONTROL_ACKERMANN_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CONTROL_ACKERMANN_WHEEL_BASE_MM / 1000.0,
    .max_turn_angle = CONFIG_CONTROL_ACKERMANN_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CONTROL_ACKERMANN_MAX_VELOCITY_MM_S / 1000.0,
};

static void init(context* ctx)
{
    syn_sub_init(&ctx->sub_cmd_vel, &ctx->cmd_vel, &chan_out_cmd_vel);
    syn_sub_init(&ctx->sub_joy, &ctx->joy, &chan_in_joy);
    syn_sub_init(&ctx->sub_fsm, &ctx->fsm, &chan_out_fsm);
    syn_pub_init(&ctx->pub_actuators, &ctx->actuators, &chan_out_actuators);
}

// computes rc_input from V, omega
void compute_actuators(context* ctx)
{
    double turn_angle = 0;
    double omega_fwd = 0;

    /* ackermann_steering:(L,omega,V)->(delta) */
    if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_MANUAL) {
        turn_angle = ctx->max_turn_angle * ctx->joy.axes[JOY_AXES_ROLL];
        omega_fwd = 0.25 * ctx->max_velocity * ctx->joy.axes[JOY_AXES_THRUST] / ctx->wheel_radius;
    } else {
        double V = ctx->cmd_vel.linear.x;
        double omega = ctx->cmd_vel.angular.z;
        casadi_int* iw = NULL;
        casadi_real* w = NULL;
        int mem = 0;
        double delta = 0;
        const casadi_real* args[3];
        casadi_real* res[1];
        args[0] = &ctx->wheel_base;
        args[1] = &omega;
        args[2] = &V;
        res[0] = &delta;
        ackermann_steering(args, res, iw, w, mem);
        omega_fwd = V / ctx->wheel_radius;
        if (fabs(V) > 0.01) {
            turn_angle = delta;
        }
    }
    if (ctx->fsm.armed != synapse_msgs_Fsm_Armed_ARMED) {
        omega_fwd = 0;
        turn_angle = 0;
    }

    synapse_msgs_Actuators* msg = &ctx->actuators;
    msg->has_header = true;
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    msg->header.seq++;
    msg->header.has_stamp = true;
    msg->header.stamp.sec = sec;
    msg->header.stamp.nanosec = nanosec;
    strncpy(msg->header.frame_id, "map", sizeof(msg->header.frame_id) - 1);

    msg->position_count = 1;
    msg->velocity_count = 1;
    msg->normalized_count = 2;
    msg->position[0] = turn_angle;
    msg->velocity[0] = omega_fwd;
#ifdef CONFIG_BUGGY3_MOTOR_ENB_REQUIRED
    msg->normalized[0] = -1;
#endif
}

static void stop(context* ctx)
{
    ctx->cmd_vel.linear.x = 0;
    ctx->cmd_vel.angular.z = 0;
}

static void run(context* ctx)
{
    init(ctx);

    while (true) {

        int ret = syn_sub_poll(&ctx->sub_cmd_vel, K_MSEC(1000));

        // claim subs/pubs
        syn_sub_claim(&ctx->sub_joy, K_MSEC(1));
        syn_sub_claim(&ctx->sub_fsm, K_MSEC(1));
        syn_sub_claim(&ctx->sub_cmd_vel, K_MSEC(1));
        syn_pub_claim(&ctx->pub_actuators, K_MSEC(1));

        if (ret != 0) {
            LOG_DBG("not receiving cmd_vel");
            // overrides cmd_vel, sets to zero
            stop(ctx);
        }

        // compute actuators based on cmd_vel
        compute_actuators(ctx);

        // release subs/pubs
        syn_sub_claim(&ctx->sub_joy, K_MSEC(1));
        syn_sub_claim(&ctx->sub_fsm, K_MSEC(1));
        syn_sub_claim(&ctx->sub_cmd_vel, K_MSEC(1));
        syn_pub_claim(&ctx->pub_actuators, K_MSEC(1));

        // publish
        syn_pub_publish(&ctx->pub_actuators, K_MSEC(1));
    }
}

K_THREAD_DEFINE(control_ackermann_vel, MY_STACK_SIZE,
    run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_ackermann_vel_callback(const struct zbus_channel* chan)
{
    syn_sub_listen(&g_ctx.sub_joy, chan, K_MSEC(1));
    syn_sub_listen(&g_ctx.sub_fsm, chan, K_MSEC(1));
    syn_sub_listen(&g_ctx.sub_cmd_vel, chan, K_MSEC(1));
}
ZBUS_LISTENER_DEFINE(listener_control_ackermann_vel, listener_control_ackermann_vel_callback);
ZBUS_CHAN_ADD_OBS(chan_in_joy, listener_control_ackermann_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_out_fsm, listener_control_ackermann_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_out_cmd_vel, listener_control_ackermann_vel, 1);

/* vi: ts=4 sw=4 et */
