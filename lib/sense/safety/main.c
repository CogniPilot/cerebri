/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <synapse_protobuf/safety.pb.h>
#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

extern struct k_work_q g_low_priority_work_q;

LOG_MODULE_REGISTER(sense_safety, CONFIG_CEREBRI_SENSE_SAFETY_LOG_LEVEL);

static void safety_toggle_work_handler(struct k_work* work);
static void safety_timer_work_handler(struct k_work* work);

typedef struct context {
    struct k_work toggle_work_item;
    struct k_work timer_work_item;
    synapse_msgs_Safety data;
    struct zros_node node;
    struct zros_pub pub;
} context_t;

static context_t g_ctx = {
    .toggle_work_item = Z_WORK_INITIALIZER(safety_toggle_work_handler),
    .timer_work_item = Z_WORK_INITIALIZER(safety_timer_work_handler),
    .data = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default },
        .status = synapse_msgs_Safety_Status_SAFETY_SAFE,
    },
    .node = {},
    .pub = {},
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(safety_button), gpios);
static struct gpio_callback button_cb_data;

static void safety_toggle_work_handler(struct k_work* work)
{
    context_t* ctx = CONTAINER_OF(work, context_t, toggle_work_item);
    synapse_msgs_Safety_Status status = ctx->data.status;
    if (status == synapse_msgs_Safety_Status_SAFETY_SAFE) {
        ctx->data.status = synapse_msgs_Safety_Status_SAFETY_UNSAFE;
    } else {
        ctx->data.status = synapse_msgs_Safety_Status_SAFETY_SAFE;
    }
    stamp_header(&ctx->data.header, k_uptime_ticks());
    ctx->data.header.seq++;
    zros_pub_update(&ctx->pub);
}

static void safety_timer_work_handler(struct k_work* work)
{
    context_t* ctx = CONTAINER_OF(work, context_t, timer_work_item);
    stamp_header(&ctx->data.header, k_uptime_ticks());
    ctx->data.header.seq++;
    zros_pub_update(&ctx->pub);
}
K_WORK_DEFINE(safety_timer_work, safety_timer_work_handler);

static void button_pressed(const struct device* dev, struct gpio_callback* cb,
    uint32_t pins)
{
    int val = gpio_pin_get_dt(&button);
    if (val > 0) {
        k_work_submit_to_queue(&g_low_priority_work_q, &g_ctx.toggle_work_item);
    }
}

static int sense_safety_init(context_t* ctx)
{
    zros_node_init(&ctx->node, "sense_safety");
    zros_pub_init(&ctx->pub, &ctx->node, &topic_safety, &ctx->data);

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
    k_work_submit_to_queue(&g_low_priority_work_q, &g_ctx.timer_work_item);
}

K_TIMER_DEFINE(safety_timer, safety_timer_handler, NULL);

static int sense_safety_entry_point(context_t* ctx)
{
    LOG_INF("init");
    sense_safety_init(ctx);
    k_timer_start(&safety_timer, K_MSEC(1000), K_MSEC(1000));
    return 0;
}

K_THREAD_DEFINE(sense_safety, MY_STACK_SIZE,
    sense_safety_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
