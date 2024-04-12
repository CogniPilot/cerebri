/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
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
#define STATE_ANY -1

LOG_MODULE_REGISTER(elm4_fsm, CONFIG_CEREBRI_ELM4_LOG_LEVEL);

void transition(
    void* state,
    bool request,
    const char* request_name,
    int pre,
    uint8_t post,
    char* buf,
    size_t n,
    size_t n_guards,
    ...)
{
    int* state_int = (int*)state;

    // not requested
    if (!request) {
        return;
    }

    // null transition
    if (pre == post) {
        LOG_DBG("null transition");
        return;
    }

    // pre state required and not matched
    if (pre >= 0 && *state_int != pre) {
        LOG_DBG("pre-state not matched");
        return;
    }

    va_list ap;
    va_start(ap, n_guards);
    for (size_t i = 0; i < n_guards; i++) {
        int guard = va_arg(ap, int);
        const char* guard_txt = va_arg(ap, const char*);
        if (guard) {
            snprintf(buf, n, "deny: %s %s", request_name, guard_txt);
            LOG_WRN("%s", buf);
            return;
        }
    }
    va_end(ap);
    snprintf(buf, n, "allow: %s", request_name);
    LOG_INF("%s", buf);
    *state_int = post;
}

typedef struct status_input_s {
    bool request_arm;
    bool request_disarm;
    bool request_manual;
    bool request_auto;
    bool request_cmd_vel;
    bool request_calibration;
    bool safe;
    bool mode_set;
    bool fuel_low;
    bool fuel_critical;
} status_input_t;

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Joy joy;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Status status;
    status_input_t status_input;
    struct zros_sub sub_joy, sub_battery_state, sub_safety;
    struct zros_pub pub_status;
} context;

static context g_ctx = {
    .node = {},
    .joy = synapse_msgs_Joy_init_default,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .status = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default,
        },
        .arming = synapse_msgs_Status_Arming_ARMING_DISARMED,
        .fuel = synapse_msgs_Status_Fuel_FUEL_UNKNOWN,
        .fuel_percentage = 0,
        .joy = synapse_msgs_Status_Joy_JOY_UNKNOWN,
        .mode = synapse_msgs_Status_Mode_MODE_UNKNOWN,
        .power = 0.0,
        .safety = synapse_msgs_Status_Safety_SAFETY_UNKNOWN,
        .status_message = "" },
    .sub_joy = {},
    .sub_battery_state = {},
    .sub_safety = {},
    .pub_status = {},
};

static void elm4_fsm_init(context* ctx)
{
    zros_node_init(&ctx->node, "elm4_fsm");
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_sub_init(&ctx->sub_battery_state, &ctx->node,
        &topic_battery_state, &ctx->battery_state, 10);
    zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
    zros_pub_init(&ctx->pub_status, &ctx->node, &topic_status, &ctx->status);
}

static void fsm_compute_input(status_input_t* input, const context* ctx)
{
    input->request_arm = ctx->joy.buttons[JOY_BUTTON_ARM] == 1;
    input->request_disarm = ctx->joy.buttons[JOY_BUTTON_DISARM] == 1;
    input->request_manual = ctx->joy.buttons[JOY_BUTTON_MANUAL] == 1;
    input->request_auto = ctx->joy.buttons[JOY_BUTTON_AUTO] == 1;
    input->request_cmd_vel = ctx->joy.buttons[JOY_BUTTON_CMD_VEL] == 1;
    input->request_calibration = ctx->joy.buttons[JOY_BUTTON_CALIBRATION] == 1;
    input->mode_set = ctx->status.mode != synapse_msgs_Status_Mode_MODE_UNKNOWN;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
    input->safe = ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_SAFE || ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_UNKNOWN;
#else
    input->safe = false;
#endif
    input->fuel_critical = ctx->battery_state.voltage < CONFIG_CEREBRI_ELM4_BATTERY_MIN_MILLIVOLT / 1000.0;
}

static void fsm_update(synapse_msgs_Status* status, const status_input_t* input)
{
    // arm transition
    transition(
        &status->arming, // state
        input->request_arm, // request
        "request arm", // label
        synapse_msgs_Status_Arming_ARMING_DISARMED, // pre
        synapse_msgs_Status_Arming_ARMING_ARMED, // post
        status->status_message, sizeof(status->status_message), // status
        // guards
        3,
        status->mode == synapse_msgs_Status_Mode_MODE_UNKNOWN, "mode not set",
        input->safe, "safety on",
        input->fuel_critical, "fuel_critical");

    // disarm transitions
    transition(
        &status->arming, // state
        input->request_disarm, // request
        "request disarm", // label
        synapse_msgs_Status_Arming_ARMING_ARMED, // pre
        synapse_msgs_Status_Arming_ARMING_DISARMED, // post
        status->status_message, sizeof(status->status_message), 0); // status

    // disarm transitions
    transition(
        &status->arming, // state
        input->fuel_critical, // request
        "disarm fuel critical", // label
        synapse_msgs_Status_Arming_ARMING_ARMED, // pre
        synapse_msgs_Status_Arming_ARMING_DISARMED, // post
        status->status_message, sizeof(status->status_message), 0); // status

    transition(
        &status->arming, // state
        input->safe, // request
        "disarm safety engaged", // label
        synapse_msgs_Status_Arming_ARMING_ARMED, // pre
        synapse_msgs_Status_Arming_ARMING_DISARMED, // post
        status->status_message, sizeof(status->status_message), 0); // status

    // mode transitions
    transition(
        &status->mode, // state
        input->request_manual, // request
        "request mode manual", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_MANUAL, // post
        status->status_message, sizeof(status->status_message), 0); // status
                                                                    //
    transition(
        &status->mode, // state
        input->request_auto || input->request_cmd_vel, // request
        "request mode cmd_vel", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_CMD_VEL, // post
        status->status_message, sizeof(status->status_message), 0); // status

    transition(
        &status->mode, // state
        input->request_calibration, // request
        "request mode calibration", // label
        STATE_ANY, // pre
        synapse_msgs_Status_Mode_MODE_CALIBRATION, // post
        status->status_message, sizeof(status->status_message), 0); // status

    // set timestamp
    stamp_header(&status->header, k_uptime_ticks());
    status->header.seq++;
}

static void status_add_extra_info(synapse_msgs_Status* status, status_input_t* input, const context* ctx)
{
    if (input->fuel_critical) {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_CRITICAL;
    } else if (input->fuel_low) {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_LOW;
    } else {
        status->fuel = synapse_msgs_Status_Fuel_FUEL_NOMINAL;
    }
    double bat_max = CONFIG_CEREBRI_ELM4_BATTERY_MAX_MILLIVOLT / 1000.0;
    double bat_min = CONFIG_CEREBRI_ELM4_BATTERY_MIN_MILLIVOLT / 1000.0;
    status->fuel_percentage = 100 * (ctx->battery_state.voltage - bat_min) / (bat_max - bat_min);
    status->power = ctx->battery_state.voltage * ctx->battery_state.current;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
    if (ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_SAFE) {
        status->safety = synapse_msgs_Status_Safety_SAFETY_SAFE;
    } else if (ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_UNSAFE) {
        status->safety = synapse_msgs_Status_Safety_SAFETY_UNSAFE;
    } else if (ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_UNKNOWN) {
        status->safety = synapse_msgs_Status_Safety_SAFETY_UNKNOWN;
    }
#else
    status->safety = synapse_msgs_Status_Safety_SAFETY_UNSAFE;
#endif
}

static void elm4_fsm_run(void* p0, void* p1, void* p2)
{
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    elm4_fsm_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_joy),
    };

    while (true) {

        // wait for joystick input event, publish at 1 Hz regardless
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("status not receiving joy");
            ctx->status.joy = synapse_msgs_Status_Joy_JOY_LOSS;
        } else {
            ctx->status.joy = synapse_msgs_Status_Joy_JOY_NOMINAL;
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
        fsm_compute_input(&ctx->status_input, ctx);
        fsm_update(&ctx->status, &ctx->status_input);
        status_add_extra_info(&ctx->status, &ctx->status_input, ctx);
        zros_pub_update(&ctx->pub_status);
    }
}

K_THREAD_DEFINE(elm4_fsm, MY_STACK_SIZE,
    elm4_fsm_run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
