/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(control_ackermann, CONFIG_CEREBRI_CONTROL_ACKERMANN_LOG_LEVEL);

typedef struct _context {
    // node
    syn_node_t node;
    // data
    synapse_msgs_Joy joy;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Fsm fsm;
    // subscriptions
    syn_sub_t sub_joy, sub_battery_state, sub_safety;
    // publications
    syn_pub_t pub_fsm;
} context;

static context g_ctx = {
    .joy = synapse_msgs_Joy_init_default,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .fsm = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default,
        },
        .armed = synapse_msgs_Fsm_Armed_DISARMED,
        .mode = synapse_msgs_Fsm_Mode_UNKNOWN_MODE,
    },
    .sub_joy = { 0 },
    .sub_battery_state = { 0 },
    .pub_fsm = { 0 },
    .node = { 0 },
};

static void control_ackermann_fsm_init(context* ctx)
{
    syn_node_init(&ctx->node, "control_ackermann_fsm");
    syn_node_add_sub(&ctx->node, &ctx->sub_joy, &ctx->joy, &chan_joy);
    syn_node_add_sub(&ctx->node, &ctx->sub_battery_state,
        &ctx->battery_state, &chan_battery_state);
    syn_node_add_sub(&ctx->node, &ctx->sub_safety, &ctx->safety, &chan_safety);
    syn_node_add_pub(&ctx->node, &ctx->pub_fsm, &ctx->fsm, &chan_fsm);
}

#define TRANSITION(STATE, GUARD, STATE_PRE, STATE_POST, LOG) \
    if (STATE == STATE_PRE && (GUARD)) {                     \
        STATE = STATE_POST;                                  \
        LOG;                                                 \
    }

#define TRANSITION_FROM_ANY(STATE, GUARD, STATE_POST, LOG) \
    if (STATE != STATE_POST && (GUARD)) {                  \
        STATE = STATE_POST;                                \
        LOG;                                               \
    }

static void update_fsm(
    const synapse_msgs_Joy* joy,
    const synapse_msgs_BatteryState* battery_state,
    const synapse_msgs_Safety* safety,
    synapse_msgs_Fsm* fsm)
{
    // fsm boolean inputs
    bool request_arm = joy->buttons[JOY_BUTTON_ARM] == 1;
    bool request_disarm = joy->buttons[JOY_BUTTON_DISARM] == 1;
    bool request_manual = joy->buttons[JOY_BUTTON_MANUAL] == 1;
    bool request_auto = joy->buttons[JOY_BUTTON_AUTO] == 1;
    bool request_cmd_vel = joy->buttons[JOY_BUTTON_CMD_VEL] == 1;

    bool mode_set = fsm->mode != synapse_msgs_Fsm_Mode_UNKNOWN_MODE;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
    bool safe = safety->status == synapse_msgs_Safety_Status_SAFE || safety->status == synapse_msgs_Safety_Status_UNKNOWN;
#else
    bool safe = false;
#endif
    bool battery_critical = battery_state->voltage < CONFIG_CEREBRI_CONTROL_ACKERMANN_BATTERY_MIN_MILLIVOLT / 1000.0;
    bool arm_allowed = mode_set && !safe && !battery_critical;

    // feedback for user
    if (request_arm) {
        if (!mode_set) {
            LOG_WRN("cannot arm until mode selected");
        }
        if (safe) {
            LOG_WRN("cannot arm, safety engaged");
        }
        if (battery_critical) {
            LOG_WRN("cannot arm, battery critical");
        }
    }

    // arm transitions
    TRANSITION(
        fsm->armed, // state
        request_arm && arm_allowed, // guard
        synapse_msgs_Fsm_Armed_DISARMED, // pre
        synapse_msgs_Fsm_Armed_ARMED, // post
        LOG_INF("armed in mode: %s", fsm_mode_str(fsm->mode));
        LOG_INF("battery voltage: %f", battery_state->voltage););

    // disarm transitions
    TRANSITION(
        fsm->armed, // state
        request_disarm, // guard
        synapse_msgs_Fsm_Armed_ARMED, // pre
        synapse_msgs_Fsm_Armed_DISARMED, // post
        LOG_INF("disarmed");
        LOG_INF("battery voltage: %f", battery_state->voltage););

    TRANSITION(
        fsm->armed, // state
        battery_critical, // guard
        synapse_msgs_Fsm_Armed_ARMED, // pre
        synapse_msgs_Fsm_Armed_DISARMED, // post
        LOG_WRN("BATTERY CRITICAL: DISARMED"););

    TRANSITION(
        fsm->armed, // state
        safe, // guard
        synapse_msgs_Fsm_Armed_ARMED, // pre
        synapse_msgs_Fsm_Armed_DISARMED, // post
        LOG_WRN("SAFETY ENGAGED: DISAMRED"););

    // mode transitions
    TRANSITION_FROM_ANY(
        fsm->mode, // state
        request_manual, // guard
        synapse_msgs_Fsm_Mode_MANUAL, // post
        LOG_INF("mode manual"));

    TRANSITION_FROM_ANY(
        fsm->mode, // state
        request_auto || request_cmd_vel, // guard
        synapse_msgs_Fsm_Mode_CMD_VEL, // post
        LOG_INF("mode cmd_vel/ auto"));

    // set timestamp
    stamp_header(&fsm->header, k_uptime_ticks());
    fsm->header.seq++;
}

static void control_ackermann_fsm_run(context* ctx)
{
    control_ackermann_fsm_init(ctx);

    while (true) {

        // wait for joystick input event, publish at 1 Hz regardless
        RC(syn_sub_poll(&ctx->sub_joy, K_MSEC(1000)),
            LOG_DBG("fsm not receiving joy"));

        // perform processing
        syn_node_lock_all(&ctx->node, K_MSEC(1));
        update_fsm(&ctx->joy, &ctx->battery_state, &ctx->safety, &ctx->fsm);
        syn_node_publish_all(&ctx->node, K_MSEC(1));
        syn_node_unlock_all(&ctx->node);
    }
}

K_THREAD_DEFINE(control_ackermann_fsm, MY_STACK_SIZE,
    control_ackermann_fsm_run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_ackermann_fsm_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(100));
}

ZBUS_LISTENER_DEFINE(listener_control_ackermann_fsm, listener_control_ackermann_fsm_callback);
ZBUS_CHAN_ADD_OBS(chan_battery_state, listener_control_ackermann_fsm, 1);
ZBUS_CHAN_ADD_OBS(chan_joy, listener_control_ackermann_fsm, 1);
ZBUS_CHAN_ADD_OBS(chan_safety, listener_control_ackermann_fsm, 1);
/* vi: ts=4 sw=4 et */
