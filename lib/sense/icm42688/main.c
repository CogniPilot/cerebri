/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
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

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 1
#define BATCH_DURATION 50

LOG_MODULE_REGISTER(sense_icm42688, CONFIG_CEREBRI_SENSE_ICM42688_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
    struct zros_node node;
    synapse_pb_ImuQ31Array imu_q31_array;
    synapse_pb_Imu imu;
    struct zros_pub pub_imu;
    struct zros_pub pub_imu_q31_array;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

// private initialization
static struct context g_ctx = {
    .node = {},
    .pub_imu = {},
    .pub_imu_q31_array = {},
    .imu = {
        .has_stamp = true,
        .has_angular_velocity = true,
        .has_linear_acceleration = true },
    .imu_q31_array = {
        .has_stamp = true,
    },
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static int sense_icm42688_init(struct context* ctx)
{
    zros_node_init(&ctx->node, "sense_icm42688");
    zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);
    zros_pub_init(&ctx->pub_imu_q31_array, &ctx->node, &topic_imu_q31_array, &ctx->imu_q31_array);
    int rc = 0;

    rc = k_sem_take(&ctx->running, K_FOREVER);
    if (rc != 0) {
        LOG_ERR("Failed to take running");
        return rc;
    }

    LOG_INF("init");
    return rc;
}

static void sense_icm42688_fini(struct context* ctx)
{
    zros_pub_fini(&ctx->pub_imu);
    zros_pub_fini(&ctx->pub_imu_q31_array);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
    LOG_INF("fini");
}

static void sense_icm42688_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    LOG_INF("starting");
    sense_icm42688_init(ctx);

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        k_msleep(1000);
    }

    LOG_INF("finished");

    sense_icm42688_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        sense_icm42688_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "sense_icm42688");
    k_thread_start(tid);
    return 0;
}

static int sense_icm42688_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_icm42688, sense_icm42688_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_icm42688, &sub_sense_icm42688, "sense icm42688 commands", NULL);

static int sense_icm42688_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(sense_icm42688_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
