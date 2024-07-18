/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 2

LOG_MODULE_REGISTER(sense_sbus, CONFIG_CEREBRI_SENSE_SBUS_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    struct zros_pub pub_input;
    synapse_pb_Input input;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    int last_event;
    int counter;
};

static struct context g_ctx = {
    .node = {},
    .pub_input = {},
    .input = {
        .channel_count = 16,
        .channel = {} },
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .last_event = 0,
    .counter = 0
};

static void sense_sbus_init(struct context* ctx)
{
    zros_node_init(&ctx->node, "sense_sbus");
    zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input_sbus, &ctx->input);
    ctx->last_event = 0;
    ctx->counter = 0;
    k_sem_take(&ctx->running, K_FOREVER);
    LOG_INF("init");
}

static void sense_sbus_fini(struct context* ctx)
{
    zros_pub_fini(&ctx->pub_input);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
    LOG_INF("fini");
}

static void sense_sbus_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    sense_sbus_init(ctx);

    // wait for stop request
    while (k_sem_take(&ctx->running, K_MSEC(1000)) < 0)
        ;

    sense_sbus_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        sense_sbus_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "sense_sbus");
    k_thread_start(tid);
    return 0;
}

static void input_cb(struct input_event* evt)
{
    // check if still running
    if (k_sem_count_get(&g_ctx.running) != 0) {
        return;
    }

    double x0 = 1024;
    double scale = 784;

    if (evt->code > 0 && evt->code <= g_ctx.input.channel_count) {
        g_ctx.input.channel[evt->code - 1] = (evt->value - x0) / scale;
    } else {
        LOG_INF("unhandled event: %d %d %d %d", evt->code, evt->sync, evt->type, evt->value);
    }

    if (evt->code < g_ctx.last_event) {
        if (g_ctx.counter++ > 10) {
            zros_pub_update(&g_ctx.pub_input);
            g_ctx.counter = 0;
        }
    }
    g_ctx.last_event = evt->code;
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(sbus)), input_cb);

static int sense_sbus_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    ARG_UNUSED(argc);
    struct context* ctx = data;

    if (strcmp(argv[0], "start") == 0) {
        if (k_sem_count_get(&g_ctx.running) == 0) {
            shell_print(sh, "already running");
        } else {
            start(ctx);
        }
    } else if (strcmp(argv[0], "stop") == 0) {
        if (k_sem_count_get(&g_ctx.running) == 0) {
            k_sem_give(&g_ctx.running);
        } else {
            shell_print(sh, "not running");
        }
    } else if (strcmp(argv[0], "status") == 0) {
        shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
    }
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_sbus, sense_sbus_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_sbus, &sub_sense_sbus, "sense sbus args", NULL);

static int sense_sbus_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(sense_sbus_sys_init, APPLICATION, 2);

/* vi: ts=4 sw=4 et */
