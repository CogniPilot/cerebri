/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/rover.h"

#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#include "mixing.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_diffdrive);

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
    const double max_velocity;
    const double wheel_separation;
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
    .wheel_radius = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_WHEEL_BASE_MM / 1000.0,
    .max_velocity = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_MAX_VELOCITY_MM_S / 1000.0,
    .wheel_separation = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_WHEEL_SEPARATION_MM / 1000.0,
};

static void init_control_diffdrive_vel(context* ctx)
{
    LOG_DBG("init vel");
    syn_node_init(&ctx->node, "control_diffdrive_vel");
    syn_node_add_sub(&ctx->node, &ctx->sub_cmd_vel, &ctx->cmd_vel, &chan_cmd_vel);
    syn_node_add_sub(&ctx->node, &ctx->sub_fsm, &ctx->fsm, &chan_fsm);
    syn_node_add_sub(&ctx->node, &ctx->sub_actuators_manual, &ctx->actuators_manual, &chan_actuators_manual);
    syn_node_add_pub(&ctx->node, &ctx->pub_actuators, &ctx->actuators, &chan_actuators);
}

// computes rc_input from V, omega
void update_cmd_vel(context* ctx)
{
    double V = ctx->cmd_vel.linear.x;
    double omega = ctx->cmd_vel.angular.z;
    casadi_int* iw = NULL;
    casadi_real* w = NULL;
    int mem = 0;
    double Vw = 0;
    const casadi_real* args[3];
    casadi_real* res[1];
    args[0] = &ctx->wheel_base;
    args[1] = &omega;
    args[2] = &ctx->wheel_separation;
    res[0] = &Vw;
    differential_steering(args, res, iw, w, mem);
    double omega_fwd = V / ctx->wheel_radius;
    double omega_turn = Vw / ctx->wheel_radius;
    diffdrive_set_actuators(&ctx->actuators, omega_fwd, omega_turn);
}

static void stop(context* ctx)
{
    diffdrive_set_actuators(&ctx->actuators, 0, 0);
}

static void run_control_diffdrive_vel(context* ctx)
{
    init_control_diffdrive_vel(ctx);

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
            LOG_DBG("no data, stopped");
        } else if (ctx->fsm.armed != synapse_msgs_Fsm_Armed_ARMED) {
            stop(ctx);
            LOG_DBG("not armed, stopped");
        } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_MANUAL) {
            LOG_DBG("manual mode");
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

K_THREAD_DEFINE(control_diffdrive_vel, MY_STACK_SIZE,
    run_control_diffdrive_vel, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_diffdrive_vel_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}
ZBUS_LISTENER_DEFINE(listener_control_diffdrive_vel, listener_control_diffdrive_vel_callback);
ZBUS_CHAN_ADD_OBS(chan_actuators_manual, listener_control_diffdrive_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_fsm, listener_control_diffdrive_vel, 1);
ZBUS_CHAN_ADD_OBS(chan_cmd_vel, listener_control_diffdrive_vel, 1);

/* vi: ts=4 sw=4 et */
