/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(control_diffdrive, CONFIG_CEREBRI_ELM4_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Joy joy;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Fsm fsm;
    struct zros_sub sub_joy, sub_battery_state, sub_safety;
    struct zros_pub pub_fsm;
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
    .sub_joy = {},
    .sub_battery_state = {},
    .pub_fsm = {},
    .node = {},
};

static void control_diffdrive_fsm_init(context* ctx)
{
    zros_node_init(&ctx->node, "control_diffdrive_fsm");
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_sub_init(&ctx->sub_battery_state, &ctx->node,
        &topic_battery_state, &ctx->battery_state, 10);
    zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
    zros_pub_init(&ctx->pub_fsm, &ctx->node, &topic_fsm, &ctx->fsm);
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
    bool request_calibration = joy->buttons[JOY_BUTTON_CALIBRATION] == 1;

    bool mode_set = fsm->mode != synapse_msgs_Fsm_Mode_UNKNOWN_MODE;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
    bool safe = safety->status == synapse_msgs_Safety_Status_SAFE || safety->status == synapse_msgs_Safety_Status_UNKNOWN;
#else
    ARG_UNUSED(safety);
    bool safe = false;
#endif
    bool battery_critical = battery_state->voltage < CONFIG_CEREBRI_ELM4_BATTERY_MIN_MILLIVOLT / 1000.0;
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

    TRANSITION_FROM_ANY(
        fsm->mode, // state
        request_calibration && safe, // guard
        synapse_msgs_Fsm_Mode_CALIBRATION, // post
        LOG_INF("mode calibration"));

    // set timestamp
    stamp_header(&fsm->header, k_uptime_ticks());
    fsm->header.seq++;
}

static void control_diffdrive_fsm_run(void* p0, void* p1, void* p2)
{
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    control_diffdrive_fsm_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_joy),
    };

    while (true) {

        // wait for joystick input event, publish at 1 Hz regardless
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("fsm not receiving joy");
        }

        if (zros_sub_update_available(&ctx->sub_battery_state)) {
            zros_sub_update(&ctx->sub_battery_state);
        }

        if (zros_sub_update_available(&ctx->sub_safety)) {
            zros_sub_update(&ctx->sub_safety);
        }

        if (zros_sub_update_available(&ctx->sub_joy)) {
            zros_sub_update(&ctx->sub_joy);
        }

        // perform processing
        update_fsm(&ctx->joy, &ctx->battery_state, &ctx->safety, &ctx->fsm);
        zros_pub_update(&ctx->pub_fsm);
    }
}

K_THREAD_DEFINE(control_diffdrive_fsm, MY_STACK_SIZE,
    control_diffdrive_fsm_run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
