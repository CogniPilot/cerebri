/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>
#include <math.h>

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(control_ackermann_lighting, CONFIG_CEREBRI_CONTROL_ACKERMANN_LOG_LEVEL);

extern struct k_work_q g_low_priority_work_q;
void static lighting_work_handler(struct k_work* work);
void static lighting_timer_handler(struct k_timer* dummy);

typedef struct context_ {
    // work
    struct k_work work_item;
    struct k_timer timer;
    // node
    syn_node_t node;
    // data
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Fsm fsm;
    synapse_msgs_LEDArray led_array;
    // subscriptions
    syn_sub_t sub_battery_state, sub_safety, sub_fsm;
    // publications
    syn_pub_t pub_led_array;
} context_t;

static context_t g_ctx = {
    .work_item = Z_WORK_INITIALIZER(lighting_work_handler),
    .timer = Z_TIMER_INITIALIZER(g_ctx.timer, lighting_timer_handler, NULL),
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .led_array = synapse_msgs_LEDArray_init_default,
    .sub_safety = {},
    .sub_fsm = {},
    .pub_led_array = {},
    .node = {},
};

static void lighting_init(context_t* ctx)
{
    syn_node_init(&ctx->node, "lighting");
    syn_node_add_sub(&ctx->node, &ctx->sub_battery_state, &ctx->battery_state, &chan_battery_state, 1);
    syn_node_add_sub(&ctx->node, &ctx->sub_safety, &ctx->safety, &chan_safety, 1);
    syn_node_add_sub(&ctx->node, &ctx->sub_fsm, &ctx->fsm, &chan_fsm, 1);
    syn_node_add_pub(&ctx->node, &ctx->pub_led_array, &ctx->led_array, &chan_led_array);
}

static void set_led(const int index, const double* color, const double brightness, synapse_msgs_LED* led)
{
    led->index = index;
    led->r = brightness * color[0];
    led->g = brightness * color[1];
    led->b = brightness * color[2];
}

static void lighting_work_handler(struct k_work* work)
{
    context_t* ctx = CONTAINER_OF(work, context_t, work_item);

    // lock node
    syn_node_lock_all(&ctx->node, K_MSEC(100));

    double t = (double)k_uptime_ticks() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    const double led_pulse_freq = 0.25;
    const double brightness_min = 4;
    const double brightness_max = 30;
    const double brightness_amplitude = (brightness_max - brightness_min) / 2;
    const double brightness_mean = (brightness_max + brightness_min) / 2;

    int brightness = brightness_mean + brightness_amplitude * sin(2 * 3.14159 * led_pulse_freq * t);
    int led_msg_index = 0;

    const int mode_leds[] = { 2, 3 };
    const double color_auto[] = { 1, 0, 0 };
    const double color_manual[] = { 0, 1, 0 };
    const double color_cmd_vel[] = { 0, 0, 1 };
    const double color_unknown[] = { 0.33, 0.33, 0.33 };

    const int arm_leds[] = { 1, 4 };
    const double color_armed[] = { 1, 0, 0 };
    const double color_disarmed[] = { 0, 1, 0 };

    const int safety_leds[] = { 0, 5 };
    const double color_unsafe[] = { 1, 0, 0 };
    const double color_safe[] = { 0, 1, 0 };
    const double color_battery_critical[] = { 1, 0.65, 0 };
    const double color_calibration[] = { 1, 1, 0 };

    bool battery_critical = ctx->battery_state.voltage < CONFIG_CEREBRI_CONTROL_ACKERMANN_BATTERY_MIN_MILLIVOLT / 1000.0;

    // mode leds
    for (int i = 0; i < sizeof(mode_leds) / sizeof(mode_leds[0]); i++) {
        const double* color = NULL;
        if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_MANUAL) {
            color = color_manual;
        } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_CMD_VEL) {
            color = color_cmd_vel;
        } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_AUTO) {
            color = color_auto;
        } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_CALIBRATION) {
            color = color_calibration;
        } else {
            color = color_unknown;
        }
        set_led(mode_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // arm leds
    for (int i = 0; i < sizeof(arm_leds) / sizeof(arm_leds[0]); i++) {
        const double* color = NULL;
        if (battery_critical) {
            color = color_battery_critical;
        } else {
            if (ctx->fsm.armed == synapse_msgs_Fsm_Armed_DISARMED) {
                color = color_disarmed;
            } else if (ctx->fsm.armed == synapse_msgs_Fsm_Armed_ARMED) {
                color = color_armed;
            } else {
                color = color_unknown;
            }
        }
        set_led(arm_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // safety leds
    for (int i = 0; i < sizeof(safety_leds) / sizeof(safety_leds[0]); i++) {
        const double* color = NULL;
        if (ctx->safety.status == synapse_msgs_Safety_Status_SAFE) {
            color = color_safe;
        } else if (ctx->safety.status == synapse_msgs_Safety_Status_UNSAFE) {
            color = color_unsafe;
        } else {
            color = color_unknown;
        }
        set_led(safety_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // set timestamp
    stamp_header(&ctx->led_array.header, k_uptime_ticks());
    ctx->led_array.header.seq++;
    ctx->led_array.led_count = led_msg_index;

    syn_node_publish_all(&ctx->node, K_MSEC(100));
    syn_node_unlock_all(&ctx->node);
}

static void lighting_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(100));
}
ZBUS_LISTENER_DEFINE(listener_control_ackermann_lighting, lighting_callback);
ZBUS_CHAN_ADD_OBS(chan_battery_state, listener_control_ackermann_lighting, 1);
ZBUS_CHAN_ADD_OBS(chan_safety, listener_control_ackermann_lighting, 1);
ZBUS_CHAN_ADD_OBS(chan_fsm, listener_control_ackermann_lighting, 1);

static void lighting_timer_handler(struct k_timer* timer)
{
    context_t* ctx = CONTAINER_OF(timer, context_t, timer);
    k_work_submit_to_queue(&g_low_priority_work_q, &ctx->work_item);
}

static int lighting_entry_point(context_t* ctx)
{
    lighting_init(ctx);
    LOG_INF("initializing lighting");
    k_timer_start(&ctx->timer, K_MSEC(500), K_MSEC(500));
    return 0;
}

K_THREAD_DEFINE(control_ackermann_lighting, MY_STACK_SIZE,
    lighting_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
