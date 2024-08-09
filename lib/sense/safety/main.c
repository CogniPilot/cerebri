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

#include <synapse_pb/safety.pb.h>
#include <synapse_topic_list.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(sense_safety, CONFIG_CEREBRI_SENSE_SAFETY_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

typedef struct context {
    struct zros_node node;
    struct zros_pub pub;
    synapse_pb_Safety data;
    struct k_sem running;
    struct k_sem data_sem;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
} context_t;

static context_t g_ctx = {
    .node = {},
    .pub = {},
    .data = {
        .has_stamp = true,
        .stamp = synapse_pb_Timestamp_init_default,
        .status = synapse_pb_Safety_Status_SAFETY_SAFE,
    },
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .data_sem = Z_SEM_INITIALIZER(g_ctx.data_sem, 1, 1),
};

static void sense_safety_init(context_t* ctx)
{
    zros_node_init(&ctx->node, "sense_safety");
    zros_pub_init(&ctx->pub, &ctx->node, &topic_safety, &ctx->data);
    k_sem_take(&ctx->running, K_FOREVER);
    LOG_INF("init");
}

static void sense_safety_fini(struct context* ctx)
{
    zros_pub_fini(&ctx->pub);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
    LOG_INF("fini");
}

static void sense_safety_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    sense_safety_init(ctx);

    // publish every one seconds while not stopped
    while (k_sem_take(&ctx->running, K_MSEC(1000)) < 0) {
        int ret = k_sem_take(&ctx->data_sem, K_NO_WAIT);
        if (ret < 0) {
            continue;
        }
        zros_pub_update(&ctx->pub);
        k_sem_give(&ctx->data_sem);
    }

    sense_safety_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        sense_safety_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "sense_safety");
    k_thread_start(tid);
    return 0;
}

static void input_cb(struct input_event* evt, void * userdata)
{
    struct context* ctx = &g_ctx;

    // check if still running
    if (k_sem_count_get(&ctx->running) != 0) {
        return;
    }

    if (evt->type == INPUT_EV_KEY && evt->code == INPUT_KEY_0 && evt->value == 1) {
        int ret = k_sem_take(&ctx->data_sem, K_FOREVER);
        if (ret < 0) {
            LOG_ERR("failed to take data sem");
        } else if (ret == 0) {
            synapse_pb_Safety_Status status = ctx->data.status;
            if (status == synapse_pb_Safety_Status_SAFETY_SAFE) {
                ctx->data.status = synapse_pb_Safety_Status_SAFETY_UNSAFE;
            } else {
                ctx->data.status = synapse_pb_Safety_Status_SAFETY_SAFE;
            }
            stamp_msg(&ctx->data.stamp, k_uptime_ticks());
            zros_pub_update(&ctx->pub);
            k_sem_give(&ctx->data_sem);
        }
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PATH(gpio_keys)), input_cb, NULL);

static int sense_safety_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_safety, sense_safety_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_safety, &sub_sense_safety, "sense safety args", NULL);

static int sense_safety_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(sense_safety_sys_init, APPLICATION, 2);

/* vi: ts=4 sw=4 et */
