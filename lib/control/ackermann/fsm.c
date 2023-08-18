/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <synapse/zbus/common.h>
#include <synapse/zbus/syn_pub_sub.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(control_ackermann, CONFIG_CONTROL_ACKERMANN_LOG_LEVEL);

typedef struct _context {
    synapse_msgs_Joy joy;
    synapse_msgs_Fsm fsm;
    synapse_msgs_BatteryState battery_state;
    struct syn_sub sub_joy, sub_battery_state;
    struct syn_pub pub_fsm;
} context;

static context g_ctx = {
    .joy = synapse_msgs_Joy_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .sub_joy = { 0 },
    .sub_battery_state = { 0 },
    .pub_fsm = { 0 }
};

static void init(context* ctx)
{
    syn_sub_init(&ctx->sub_joy, &ctx->joy, &chan_in_joy);
    syn_sub_init(&ctx->sub_battery_state, &ctx->battery_state, &chan_out_battery_state);
    syn_pub_init(&ctx->pub_fsm, &ctx->fsm, &chan_out_fsm);
}

static void update_fsm(synapse_msgs_Fsm* fsm)
{
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    fsm->has_header = true;
    fsm->header.has_stamp = true;
    fsm->header.stamp.sec = sec;
    fsm->header.stamp.nanosec = nanosec;
    fsm->header.seq++;
    strncpy(fsm->header.frame_id, "base_link", sizeof(fsm->header.frame_id) - 1);
}

static void handle_joy(
    const synapse_msgs_Joy* joy,
    const synapse_msgs_BatteryState* battery_state,
    synapse_msgs_Fsm* fsm)
{
    // safety switch : TODO

    // arming
    if (joy->buttons[JOY_BUTTON_ARM] == 1 && fsm->armed != synapse_msgs_Fsm_Armed_ARMED) {
        if (fsm->mode == synapse_msgs_Fsm_Mode_UNINITIALIZED) {
            LOG_WRN("cannot arm until mode selected");
            return;
        }
        LOG_INF("armed in mode: %s", fsm_mode_str(fsm->mode));
        LOG_INF("battery voltage: %f", battery_state->voltage);
        fsm->armed = synapse_msgs_Fsm_Armed_ARMED;
    } else if (joy->buttons[JOY_BUTTON_DISARM] == 1 && fsm->armed == synapse_msgs_Fsm_Armed_ARMED) {
        LOG_INF("disarmed");
        fsm->armed = synapse_msgs_Fsm_Armed_DISARMED;
    }

    // handle modes
    synapse_msgs_Fsm_Mode prev_mode = fsm->mode;
    if (joy->buttons[JOY_BUTTON_MANUAL] == 1) {
        fsm->mode = synapse_msgs_Fsm_Mode_MANUAL;
    } else if (joy->buttons[JOY_BUTTON_AUTO] == 1) {
        fsm->mode = synapse_msgs_Fsm_Mode_AUTO;
    } else if (joy->buttons[JOY_BUTTON_CMD_VEL] == 1) {
        fsm->mode = synapse_msgs_Fsm_Mode_CMD_VEL;
    }

    // notify on mode change
    if (fsm->mode != prev_mode) {
        LOG_INF("mode changed to: %s", fsm_mode_str(fsm->mode));
    }

    // publish relevant joystick commands
    if (fsm->mode == synapse_msgs_Fsm_Mode_MANUAL) {
        // zbus_chan_pub(&chan_out_cmd_vel, &cmd_vel, K_NO_WAIT);
    }

    // forwarding of chan_in_joy to chan_out_joy is done in listener
    // if mode is CMD_VEL
}

static void run(context* ctx)
{
    init(ctx);

    while (true) {

        // wait for joystick input event
        int ret = syn_sub_poll(&ctx->sub_joy, K_MSEC(100));

        // if no joystick event received
        if (ret != 0) {
            LOG_DBG("not receiving manual control signal");
        }

        // lock data
        syn_sub_claim(&ctx->sub_joy, K_MSEC(1));
        syn_sub_claim(&ctx->sub_battery_state, K_MSEC(1));
        syn_pub_claim(&ctx->pub_fsm, K_MSEC(1));

        // perform processing
        handle_joy(&ctx->joy, &ctx->battery_state, &ctx->fsm);
        update_fsm(&ctx->fsm);

        // unlock data
        syn_sub_finish(&ctx->sub_joy);
        syn_sub_finish(&ctx->sub_battery_state);
        syn_pub_finish(&ctx->pub_fsm);

        // publish channel
        syn_pub_publish(&ctx->pub_fsm, K_MSEC(1));
    }
}

K_THREAD_DEFINE(control_ackermann_fsm, MY_STACK_SIZE,
    run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

static void listener_control_ackermann_fsm_callback(const struct zbus_channel* chan)
{
    syn_sub_listen(&g_ctx.sub_joy, chan, K_MSEC(1));
    syn_sub_listen(&g_ctx.sub_battery_state, chan, K_MSEC(1));
}

ZBUS_LISTENER_DEFINE(listener_control_ackermann_fsm, listener_control_ackermann_fsm_callback);
ZBUS_CHAN_ADD_OBS(chan_out_battery_state, listener_control_ackermann_fsm, 1);
ZBUS_CHAN_ADD_OBS(chan_in_joy, listener_control_ackermann_fsm, 1);

/* vi: ts=4 sw=4 et */
