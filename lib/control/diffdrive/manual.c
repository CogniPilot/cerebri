/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#include "mixing.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_diffdrive);

typedef struct _context {
    syn_node_t node;
    synapse_msgs_Joy joy;
    synapse_msgs_Actuators actuators_manual;
    syn_sub_t sub_joy;
    syn_pub_t pub_actuators_manual;
    const double wheel_radius;
    const double max_angular_velocity;
    const double max_velocity;
} context;

static context g_ctx = {
    .node = { 0 },
    .joy = synapse_msgs_Joy_init_default,
    .actuators_manual = synapse_msgs_Actuators_init_default,
    .sub_joy = { 0 },
    .pub_actuators_manual = { 0 },
    .wheel_radius = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_WHEEL_RADIUS_MM / 1000.0,
    .max_angular_velocity = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_MAX_ANGULAR_VELOCITY_MRAD_S / 1000.0,
    .max_velocity = CONFIG_CEREBRI_CONTROL_DIFFDRIVE_MAX_VELOCITY_MM_S / 1000.0,
};

static void init(context* ctx)
{
    syn_node_init(&ctx->node, "manual");
    syn_node_add_sub(&ctx->node, &ctx->sub_joy, &ctx->joy, &chan_joy, 10);
    syn_node_add_pub(&ctx->node, &ctx->pub_actuators_manual,
        &ctx->actuators_manual, &chan_actuators_manual);
}

static void run(context* ctx)
{
    init(ctx);

    while (true) {
        syn_sub_poll(&ctx->sub_joy, K_MSEC(1000));
        syn_node_lock_all(&g_ctx.node, K_MSEC(1));

        // compute turn_angle, and angular velocity from joystick
        double omega_turn = ctx->max_angular_velocity * ctx->joy.axes[JOY_AXES_ROLL];
        double omega_fwd = ctx->max_velocity * ctx->joy.axes[JOY_AXES_THRUST] / ctx->wheel_radius;
        diffdrive_set_actuators(&ctx->actuators_manual, omega_fwd, omega_turn);

        syn_node_publish_all(&g_ctx.node, K_MSEC(1));
        syn_node_unlock_all(&g_ctx.node);
    }
}

K_THREAD_DEFINE(control_diffdrive_manual, MY_STACK_SIZE,
    run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_diffdrive_manual_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(1));
}

ZBUS_LISTENER_DEFINE(listener_control_diffdrive_manual, listener_control_diffdrive_manual_callback);
ZBUS_CHAN_ADD_OBS(chan_joy, listener_control_diffdrive_manual, 1);

/* vi: ts=4 sw=4 et */
