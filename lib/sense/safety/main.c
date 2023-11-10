/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <cerebri/synapse/zbus/common.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

extern struct k_work_q g_low_priority_work_q;

LOG_MODULE_REGISTER(sense_safety, CONFIG_CEREBRI_SENSE_SAFETY_LOG_LEVEL);

typedef struct _context {
    synapse_msgs_Safety safety;
    syn_pub_t pub_safety;
} context;

static context g_ctx = {
    .safety = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default },
        .status = synapse_msgs_Safety_Status_SAFE,
    },
    .pub_safety = { 0 }
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(safety_button), gpios);
static struct gpio_callback button_cb_data;

static void safety_toggle_work_handler(struct k_work* work)
{
    if (syn_pub_lock(&g_ctx.pub_safety, K_MSEC(1)) != 0)
        return;

    synapse_msgs_Safety_Status status = g_ctx.safety.status;
    if (status == synapse_msgs_Safety_Status_SAFE) {
        g_ctx.safety.status = synapse_msgs_Safety_Status_UNSAFE;
    } else {
        g_ctx.safety.status = synapse_msgs_Safety_Status_SAFE;
    }
    stamp_header(&g_ctx.safety.header, k_uptime_ticks());
    g_ctx.safety.header.seq += 1;
    syn_pub_unlock(&g_ctx.pub_safety);
    syn_pub_publish(&g_ctx.pub_safety, K_MSEC(100));
}
K_WORK_DEFINE(safety_toggle_work, safety_toggle_work_handler);

static void safety_timer_work_handler(struct k_work* work)
{
    if (syn_pub_lock(&g_ctx.pub_safety, K_MSEC(1)) != 0)
        return;
    stamp_header(&g_ctx.safety.header, k_uptime_ticks());
    g_ctx.safety.header.seq += 1;
    syn_pub_unlock(&g_ctx.pub_safety);
    syn_pub_publish(&g_ctx.pub_safety, K_MSEC(100));
}
K_WORK_DEFINE(safety_timer_work, safety_timer_work_handler);

static void button_pressed(const struct device* dev, struct gpio_callback* cb,
    uint32_t pins)
{
    k_work_submit_to_queue(&g_low_priority_work_q, &safety_toggle_work);
}

static int sense_safety_init(context* ctx)
{
    syn_pub_init(&ctx->pub_safety, &ctx->safety, &chan_safety);

    int ret;

    // schedule button interrupt to update state and publish
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("button device %s is not ready\n",
            button.port->name);
    }
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("%d: failed to configure %s pin %d\n",
            ret, button.port->name, button.pin);
    }
    ret = gpio_pin_interrupt_configure_dt(&button,
        GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("%d: failed to configure interrupt on %s pin %d\n",
            ret, button.port->name, button.pin);
    }
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("Set up button at %s pin %d\n", button.port->name, button.pin);
    return ret;
}

void safety_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_low_priority_work_q, &safety_timer_work);
}

K_TIMER_DEFINE(safety_timer, safety_timer_handler, NULL);

static int sense_safety_entry_point(context* ctx)
{
    sense_safety_init(ctx);
    k_timer_start(&safety_timer, K_MSEC(1000), K_MSEC(1000));
    return 0;
}

K_THREAD_DEFINE(sense_safety, MY_STACK_SIZE,
    sense_safety_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
