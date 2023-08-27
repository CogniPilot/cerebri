/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#include "common.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_ackermann);

typedef struct _context {
    syn_node_t node;
    synapse_msgs_Joy joy;
    synapse_msgs_Actuators actuators_manual;
    syn_sub_t sub_joy;
    syn_pub_t pub_actuators_manual;
    const double wheel_radius;
    const double max_turn_angle;
    const double max_velocity;
} context;

static context g_ctx = {
    .node = { 0 },
    .joy = synapse_msgs_Joy_init_default,
    .actuators_manual = synapse_msgs_Actuators_init_default,
    .sub_joy = { 0 },
    .pub_actuators_manual = { 0 },
    .wheel_radius = CONFIG_CEREBRI_CONTROL_ACKERMANN_WHEEL_RADIUS_MM / 1000.0,
    .max_turn_angle = CONFIG_CEREBRI_CONTROL_ACKERMANN_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CEREBRI_CONTROL_ACKERMANN_MAX_VELOCITY_MM_S / 1000.0,
};

static void init(context* ctx)
{
    syn_node_init(&ctx->node, "manual");
    syn_node_add_sub(&ctx->node, &ctx->sub_joy, &ctx->joy, &chan_in_joy);
    syn_node_add_pub(&ctx->node, &ctx->pub_actuators_manual,
        &ctx->actuators_manual, &chan_out_actuators_manual);
}

static void run(context* ctx)
{
    init(ctx);

    while (true) {
        syn_sub_poll(&ctx->sub_joy, K_MSEC(1000));
        syn_node_lock_all(&g_ctx.node, K_MSEC(1));

        // compute turn_angle, and angular velocity from joystick
        double turn_angle = ctx->max_turn_angle * ctx->joy.axes[JOY_AXES_ROLL];
        double omega_fwd = 0.1 * ctx->max_velocity * ctx->joy.axes[JOY_AXES_THRUST] / ctx->wheel_radius;
        ackermann_set_actuators(&ctx->actuators_manual, turn_angle, omega_fwd);

        syn_node_publish_all(&g_ctx.node, K_MSEC(1));
        syn_node_unlock_all(&g_ctx.node);
    }
}

K_THREAD_DEFINE(control_ackermann_manual, MY_STACK_SIZE,
    run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_ackermann_manual_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}

ZBUS_LISTENER_DEFINE(listener_control_ackermann_manual, listener_control_ackermann_manual_callback);
ZBUS_CHAN_ADD_OBS(chan_in_joy, listener_control_ackermann_manual, 1);

/* vi: ts=4 sw=4 et */
