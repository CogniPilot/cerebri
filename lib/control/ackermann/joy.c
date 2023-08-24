/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <synapse/zbus/common.h>
#include <synapse/zbus/syn_pub_sub.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_DECLARE(control_ackermann);

typedef struct _context {
    synapse_msgs_Joy joy;
    struct syn_pub pub_joy;
} context;

static context g_ctx = {
    .joy = synapse_msgs_Joy_init_default,
    .pub_joy = { 0 }
};

static void init(context* ctx)
{
    syn_pub_init(&ctx->pub_joy, &ctx->joy, &chan_in_joy);
}

static void run(context* ctx)
{
    init(ctx);

    while (true) {
        k_msleep(100);

        syn_pub_claim(&ctx->pub_joy, K_MSEC(1));

        ctx->joy.axes_count = 2;
        ctx->joy.axes[0] = 0.2;
        ctx->joy.axes[1] = 0.2;

        syn_pub_finish(&ctx->pub_joy);

        syn_pub_publish(&ctx->pub_joy, K_MSEC(1));
    }
}

K_THREAD_DEFINE(control_pub_joy, MY_STACK_SIZE,
    run, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
