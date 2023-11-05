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

#include <cerebri/synapse/zbus/channels.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#define DELAY_TIME K_MSEC(40)

LOG_MODULE_REGISTER(actuate_led_array, CONFIG_CEREBRI_ACTUATE_LED_ARRAY_LOG_LEVEL);

extern struct k_work_q g_low_priority_work_q;

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

typedef struct _context {
    // node
    syn_node_t node;
    // data
    synapse_msgs_LEDArray led_array;
    // subscriptions
    syn_sub_t sub_led_array;
    // devices
    const struct device* strip;
    struct led_rgb strip_colors[CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT];
} context;

static context g_ctx = {
    .node = {},
    .led_array = synapse_msgs_LEDArray_init_default,
    .sub_led_array = {},
    .strip = NULL,
    .strip_colors = {},
};

static void actuate_led_array_init(context* ctx)
{
    // initialize node
    syn_node_init(&ctx->node, "actuate_led_array");
    syn_node_add_sub(&ctx->node,
        &ctx->sub_led_array, &ctx->led_array, &chan_in_led_array);

    g_ctx.strip = DEVICE_DT_GET_ANY(apa102);
    if (!g_ctx.strip) {
        LOG_ERR("LED strip device not found");
        return;
    } else if (!device_is_ready(g_ctx.strip)) {
        LOG_ERR("LED strip device %s is not ready", g_ctx.strip->name);
        return;
    } else {
        LOG_INF("Found LED strip device %s", g_ctx.strip->name);
    }
}

static void listener_actuate_led_array_callback(const struct zbus_channel* chan)
{
    syn_node_listen(&g_ctx.node, chan, K_MSEC(100));
}

ZBUS_LISTENER_DEFINE(listener_actuate_led_array, listener_actuate_led_array_callback);
ZBUS_CHAN_ADD_OBS(chan_in_led_array, listener_actuate_led_array, 1);

void actuate_led_array_entry_point(context* ctx)
{
    actuate_led_array_init(ctx);

    while (true) {
        RC(syn_sub_poll(&ctx->sub_led_array, K_MSEC(1000)),
            LOG_DBG("not receiving led_array"));

        // perform processing
        syn_node_lock_all(&ctx->node, K_MSEC(1));
        for (int i = 0; i < ctx->led_array.led_count; i++) {
            synapse_msgs_LED led = ctx->led_array.led[i];
            if (led.index > CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT) {
                LOG_ERR("Setting LED index out of range");
                continue;
            }
            ctx->strip_colors[led.index].r = led.r;
            ctx->strip_colors[led.index].g = led.g;
            ctx->strip_colors[led.index].b = led.b;
        }
        led_strip_update_rgb(ctx->strip, ctx->strip_colors, CONFIG_CEREBRI_ACTUATE_LED_ARRAY_COUNT);
        syn_node_unlock_all(&ctx->node);
    }
}

K_THREAD_DEFINE(actuate_led_array, MY_STACK_SIZE,
    actuate_led_array_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
