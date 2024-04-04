/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 6

LOG_MODULE_REGISTER(rdd2_lighting, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
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
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety = synapse_msgs_Safety_init_default,
    .status = synapse_msgs_Status_init_default,
    .led_array = synapse_msgs_LEDArray_init_default,
    .sub_safety = {},
    .sub_status = {},
    .sub_joy = {},
    .pub_led_array = {},
    .lights_on = false,
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void rdd2_lighting_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_lighting");
    zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state, &ctx->battery_state, 10);
    zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_pub_init(&ctx->pub_led_array, &ctx->node, &topic_led_array, &ctx->led_array);
    atomic_set(&ctx->running, 1);
}

static void rdd2_lighting_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_sub_fini(&ctx->sub_battery_state);
    zros_sub_fini(&ctx->sub_safety);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_joy);
    zros_pub_fini(&ctx->pub_led_array);
    atomic_set(&ctx->running, 0);
}

static void set_led(const int index, const double* color, const double brightness, synapse_msgs_LED* led)
{
    led->index = index;
    led->r = brightness * color[0];
    led->g = brightness * color[1];
    led->b = brightness * color[2];
}

static void rdd2_lighting_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_lighting_init(ctx);

    while (atomic_get(&ctx->running)) {

        // wait 33 ms
        k_msleep(33);

        // update subscriptions
        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_joy)) {
            zros_sub_update(&ctx->sub_joy);
        }

        if (zros_sub_update_available(&ctx->sub_safety)) {
            zros_sub_update(&ctx->sub_safety);
        }

        if (zros_sub_update_available(&ctx->sub_battery_state)) {
            zros_sub_update(&ctx->sub_battery_state);
        }

        // timing
        double t = k_uptime_ticks() / ((double)CONFIG_SYS_CLOCK_TICKS_PER_SEC);
        const double led_pulse_freq = 0.25;
        const double brightness_min = 4;
        const double brightness_max = 30;
        const double brightness_amplitude = (brightness_max - brightness_min) / 2;
        const double brightness_mean = (brightness_max + brightness_min) / 2;

        int brightness = brightness_mean + brightness_amplitude * sin(2 * 3.14159 * led_pulse_freq * t);
        int led_msg_index = 0;

        const int mode_leds[] = { 2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35 };
        const double color_auto[] = { 1, 0, 0 };
        const double color_manual[] = { 0, 1, 0 };
        const double color_cmd_vel[] = { 0, 0, 1 };
        const double color_unknown[] = { 0.33, 0.33, 0.33 };

        const int arm_leds[] = { 1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34 };
        const double color_armed[] = { 1, 0, 0 };
        const double color_disarmed[] = { 0, 1, 0 };

        const int safety_leds[] = { 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33 };
        const double color_unsafe[] = { 1, 0, 0 };
        const double color_safe[] = { 0, 1, 0 };
        const double color_battery_critical[] = { 1, 0.65, 0 };
        const double color_calibration[] = { 1, 1, 0 };

        bool battery_critical = ctx->battery_state.voltage < CONFIG_CEREBRI_RDD2_BATTERY_MIN_MILLIVOLT / 1000.0;

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


        // set timestamp
        stamp_header(&ctx->led_array.header, k_uptime_ticks());
        ctx->led_array.header.seq++;
        ctx->led_array.led_count = led_msg_index;

        zros_pub_update(&ctx->pub_led_array);
    }

    rdd2_lighting_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_lighting_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_lighting");
    k_thread_start(tid);
    return 0;
}

static int rdd2_lighting_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct context* ctx = data;
    assert(argc == 1);

    if (strcmp(argv[0], "start") == 0) {
        if (atomic_get(&ctx->running)) {
            shell_print(sh, "already running");
        } else {
            start(ctx);
        }
    } else if (strcmp(argv[0], "stop") == 0) {
        if (atomic_get(&ctx->running)) {
            atomic_set(&ctx->running, 0);
        } else {
            shell_print(sh, "not running");
        }
    } else if (strcmp(argv[0], "status") == 0) {
        shell_print(sh, "running: %d", (int)atomic_get(&ctx->running));
    }
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_lighting, rdd2_lighting_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_lighting, &sub_rdd2_lighting, "rdd2 lighting commands", NULL);

static int rdd2_lighting_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_lighting_sys_init, APPLICATION, 10);

// vi: ts=4 sw=4 et
