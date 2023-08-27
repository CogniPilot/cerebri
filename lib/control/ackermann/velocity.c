/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/rover.h"
#include <math.h>

#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#include "common.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_ackermann);

typedef struct _context {
    syn_node_t node;
    synapse_msgs_Twist cmd_vel;
    synapse_msgs_Fsm fsm;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Actuators actuators_manual;
    syn_sub_t sub_fsm, sub_cmd_vel, sub_actuators_manual;
    syn_pub_t pub_actuators;
    const double wheel_radius;
    const double wheel_base;
    const double max_turn_angle;
    const double max_velocity;
} context;

static context g_ctx = {
    .node = { 0 },
    .cmd_vel = synapse_msgs_Twist_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .actuators_manual = synapse_msgs_Actuators_init_default,
    .sub_fsm = { 0 },
    .sub_cmd_vel = { 0 },
    .sub_actuators_manual = { 0 },
    .pub_actuators = { 0 },
    .wheel_radius = CONFIG_CEREBRI_CONTROL_ACKERMANN_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CEREBRI_CONTROL_ACKERMANN_WHEEL_BASE_MM / 1000.0,
    .max_turn_angle = CONFIG_CEREBRI_CONTROL_ACKERMANN_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CEREBRI_CONTROL_ACKERMANN_MAX_VELOCITY_MM_S / 1000.0,
};

static void init_control_ackermann_vel(context* ctx)
{
    LOG_DBG("init vel");
    syn_node_init(&ctx->node, "control_ackerman_vel");
    syn_node_add_sub(&ctx->node, &ctx->sub_cmd_vel, &ctx->cmd_vel, &chan_out_cmd_vel);
    syn_node_add_sub(&ctx->node, &ctx->sub_fsm, &ctx->fsm, &chan_out_fsm);
    syn_node_add_sub(&ctx->node, &ctx->sub_actuators_manual, &ctx->actuators_manual, &chan_out_actuators_manual);
    syn_node_add_pub(&ctx->node, &ctx->pub_actuators, &ctx->actuators, &chan_out_actuators);
}

// computes rc_input from V, omega
void update_cmd_vel(context* ctx)
{
    double turn_angle = 0;
    double omega_fwd = 0;
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
    ackermann_set_actuators(&ctx->actuators, turn_angle, omega_fwd);
}

static void stop(context* ctx)
{
    ackermann_set_actuators(&ctx->actuators, 0, 0);
}

static void run_control_ackermann_vel(context* ctx)
{
    init_control_ackermann_vel(ctx);

    while (true) {
        int ret = 0;
        syn_node_lock_all(&ctx->node, K_MSEC(1));
        synapse_msgs_Fsm_Mode mode = ctx->fsm.mode;
        syn_node_unlock_all(&ctx->node);

        if (mode == synapse_msgs_Fsm_Mode_MANUAL) {
            ret = syn_sub_poll(&ctx->sub_actuators_manual, K_MSEC(1000));
            if (ret != 0) {
                LOG_DBG("not receiving manual actuators");
            }
        } else {
            ret = syn_sub_poll(&ctx->sub_cmd_vel, K_MSEC(1000));
            if (ret != 0) {
                LOG_DBG("not receiving cmd_vel");
            }
        }

        // lock subs/pubs
        syn_node_lock_all(&ctx->node, K_MSEC(1));

        // handle modes
        if (ret < 0) {
            stop(ctx);
            LOG_INF("no data, stopped");
        } else if (ctx->fsm.armed != synapse_msgs_Fsm_Armed_ARMED) {
            stop(ctx);
            LOG_INF("not armed, stopped");
        } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_MANUAL) {
            LOG_INF("manual mode");
            ctx->actuators = ctx->actuators_manual;
        } else {
            update_cmd_vel(ctx);
        }

        // publish
        syn_node_publish_all(&ctx->node, K_MSEC(1));

        // unlock subs/pubs
        syn_node_unlock_all(&ctx->node);
    }
}

K_THREAD_DEFINE(control_ackermann_vel, MY_STACK_SIZE,
    run_control_ackermann_vel, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_ackermann_vel_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}
ZBUS_LISTENER_DEFINE(listener_control_ackermann_vel, listener_control_ackermann_vel_callback);
ZBUS_CHAN_ADD_OBS(chan_out_actuators_manual, listener_control_ackermann_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_out_fsm, listener_control_ackermann_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_out_cmd_vel, listener_control_ackermann_vel, 1);

/* vi: ts=4 sw=4 et */
