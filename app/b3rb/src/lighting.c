/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_lighting, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

extern struct k_work_q g_low_priority_work_q;
static void lighting_work_handler(struct k_work* work);
static void lighting_timer_handler(struct k_timer* dummy);

typedef struct context_ {
    // work
    struct k_work work_item;
    struct k_timer timer;
    // node
    struct zros_node node;
    // data
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Status status;
    synapse_msgs_LEDArray led_array;
    synapse_msgs_Joy joy;
    // subscriptions
    struct zros_sub sub_battery_state, sub_safety, sub_status, sub_joy;
    // publications
    struct zros_pub pub_led_array;
    bool lights_on;
} context_t;

static context_t g_ctx = {
    .work_item = Z_WORK_INITIALIZER(lighting_work_handler),
    .timer = Z_TIMER_INITIALIZER(g_ctx.timer, lighting_timer_handler, NULL),
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .status = synapse_msgs_Status_init_default,
    .led_array = synapse_msgs_LEDArray_init_default,
    .sub_safety = {},
    .sub_status = {},
    .sub_joy = {},
    .pub_led_array = {},
    .node = {},
    .lights_on = false,
};

static void lighting_init(context_t* ctx)
{
    zros_node_init(&ctx->node, "b3rb_lighting");
    zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state, &ctx->battery_state, 10);
    zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_pub_init(&ctx->pub_led_array, &ctx->node, &topic_led_array, &ctx->led_array);
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

    // update subscriptions
    zros_sub_update(&ctx->sub_status);
    zros_sub_update(&ctx->sub_joy);
    zros_sub_update(&ctx->sub_safety);
    zros_sub_update(&ctx->sub_battery_state);

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

    const int headlight_leds[] = { 6, 7, 8, 9, 10, 11 };
    const double color_white[] = { 1, 1, 1 };

    bool battery_critical = ctx->battery_state.voltage < CONFIG_CEREBRI_B3RB_BATTERY_MIN_MILLIVOLT / 1000.0;

    // mode leds
    for (size_t i = 0; i < ARRAY_SIZE(mode_leds); i++) {
        const double* color = NULL;
        if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
            color = color_manual;
        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_CMD_VEL) {
            color = color_cmd_vel;
        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_AUTO) {
            color = color_auto;
        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_CALIBRATION) {
            color = color_calibration;
        } else {
            color = color_unknown;
        }
        set_led(mode_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // arm leds
    for (size_t i = 0; i < ARRAY_SIZE(arm_leds); i++) {
        const double* color = NULL;
        if (battery_critical) {
            color = color_battery_critical;
        } else {
            if (ctx->status.arming == synapse_msgs_Status_Arming_ARMING_DISARMED) {
                color = color_disarmed;
            } else if (ctx->status.arming == synapse_msgs_Status_Arming_ARMING_ARMED) {
                color = color_armed;
            } else {
                color = color_unknown;
            }
        }
        set_led(arm_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // safety leds
    for (size_t i = 0; i < ARRAY_SIZE(safety_leds); i++) {
        const double* color = NULL;
        if (ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_SAFE) {
            color = color_safe;
        } else if (ctx->safety.status == synapse_msgs_Safety_Status_SAFETY_UNSAFE) {
            color = color_unsafe;
        } else {
            color = color_unknown;
        }
        set_led(safety_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
        led_msg_index++;
    }

    // headlight leds
    bool lights_on_requested = ctx->joy.buttons[JOY_BUTTON_LIGHTS_ON] == 1;
    bool lights_off_requested = ctx->joy.buttons[JOY_BUTTON_LIGHTS_OFF] == 1;

    if (lights_on_requested) {
        ctx->lights_on = true;
    } else if (lights_off_requested) {
        ctx->lights_on = false;
    }

    if (ctx->lights_on) {
        for (size_t i = 0; i < ARRAY_SIZE(headlight_leds); i++) {
            set_led(headlight_leds[i], color_white, 255, &ctx->led_array.led[led_msg_index]);
            led_msg_index++;
        }
    } else if (!ctx->lights_on) {
        for (size_t i = 0; i < ARRAY_SIZE(headlight_leds); i++) {
            set_led(headlight_leds[i], color_white, 0, &ctx->led_array.led[led_msg_index]);
            led_msg_index++;
        }
    }

    // set timestamp
    stamp_header(&ctx->led_array.header, k_uptime_ticks());
    ctx->led_array.header.seq++;
    ctx->led_array.led_count = led_msg_index;

    zros_pub_update(&ctx->pub_led_array);
}

static void lighting_timer_handler(struct k_timer* timer)
{
    context_t* ctx = CONTAINER_OF(timer, context_t, timer);
    k_work_submit_to_queue(&g_low_priority_work_q, &ctx->work_item);
}

static void lighting_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context_t* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    lighting_init(ctx);
    k_timer_start(&ctx->timer, K_MSEC(33), K_MSEC(33));
}

K_THREAD_DEFINE(b3rb_lighting, MY_STACK_SIZE,
    lighting_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 1000);

/* vi: ts=4 sw=4 et */
