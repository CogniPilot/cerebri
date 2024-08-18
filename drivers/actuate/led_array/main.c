/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_pb/led_array.pb.h>
#include <synapse_topic_list.h>

#define DELAY_TIME K_MSEC(40)

LOG_MODULE_REGISTER(actuate_led_array, CONFIG_CEREBRI_ACTUATE_LED_ARRAY_LOG_LEVEL);

extern struct k_work_q g_low_priority_work_q;

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

typedef struct _context {
    struct zros_node node;
    struct zros_sub sub;
    synapse_pb_LEDArray data;
    const struct device* strip;
    struct led_rgb strip_colors[CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT];
} context;

static context g_ctx = {
    .data = synapse_pb_LEDArray_init_default,
    .node = {},
    .sub = {},
    .strip = NULL,
    .strip_colors = {},
};

static void actuate_led_array_init(context* ctx)
{
    zros_node_init(&ctx->node, "actuate_led_array");
    zros_sub_init(&ctx->sub, &ctx->node, &topic_led_array, &ctx->data, 10);
    g_ctx.strip = DEVICE_DT_GET_ANY(apa_apa102);
    if (!g_ctx.strip) {
        LOG_ERR("LED strip device not found");
        return;
    } else if (!device_is_ready(g_ctx.strip)) {
        LOG_ERR("LED strip device %s is not ready", g_ctx.strip->name);
        return;
    }
}

void actuate_led_array_entry_point(context* ctx)
{
    LOG_INF("init");
    actuate_led_array_init(ctx);

    struct k_poll_event events[2] = {
        *zros_sub_get_event(&ctx->sub),
    };

    while (true) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("sub polling error! %d", rc);
        }

        if (zros_sub_update_available(&ctx->sub)) {
            zros_sub_update(&ctx->sub);
        }

        // perform processing
        for (int i = 0; i < ctx->data.led_count; i++) {
            synapse_pb_LEDArray_LED led = ctx->data.led[i];
            if (led.index > CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT) {
                LOG_ERR("Setting LED index out of range");
                continue;
            }
            ctx->strip_colors[led.index].r = led.r;
            ctx->strip_colors[led.index].g = led.g;
            ctx->strip_colors[led.index].b = led.b;
        }
        led_strip_update_rgb(ctx->strip, ctx->strip_colors, CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT);
    }
}

K_THREAD_DEFINE(actuate_led_array, MY_STACK_SIZE,
    actuate_led_array_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
