/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/gen/rdd2.h"
#include "math.h"
#include <assert.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(rdd2_allocation, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Vector3 force_sp, moment_sp;
    struct zros_sub sub_status, sub_force_sp, sub_moment_sp;
    struct zros_pub pub_actuators;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .status = synapse_msgs_Status_init_default,
    .actuators = {
        .has_header = true,
        .header = {
            .seq = 0,
            .frame_id = "odom",
            .has_stamp = true },
        .velocity_count = 4,
    },
    .sub_status = {},
    .sub_force_sp = {},
    .sub_moment_sp = {},
    .pub_actuators = {},
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void rdd2_allocation_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_allocation");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp, 10);
    zros_sub_init(&ctx->sub_moment_sp, &ctx->node, &topic_moment_sp, &ctx->moment_sp, 100);
    zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
    atomic_set(&ctx->running, 1);
}

static void rdd2_allocation_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_force_sp);
    zros_sub_fini(&ctx->sub_moment_sp);
    zros_pub_fini(&ctx->pub_actuators);
    atomic_set(&ctx->running, 0);
}

static void stop(struct context* ctx)
{
    for (int i = 0; i < 4; i++) {
        ctx->actuators.velocity[i] = 0;
    }
}

static void rdd2_allocation_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_allocation_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_moment_sp),
    };

    while (atomic_get(&ctx->running)) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(100));
        if (rc != 0) {
            LOG_DBG("not receiving moment_sp");
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_force_sp)) {
            zros_sub_update(&ctx->sub_force_sp);
        }

        if (zros_sub_update_available(&ctx->sub_moment_sp)) {
            zros_sub_update(&ctx->sub_moment_sp);
        }

        if (rc < 0) {
            stop(ctx);
            LOG_DBG("no data, stopped");
        } else if (ctx->status.arming != synapse_msgs_Status_Arming_ARMING_ARMED) {
            stop(ctx);
            LOG_DBG("not armed, stopped");
        } else {
            double thrust = ctx->force_sp.z;
            double roll = ctx->moment_sp.x;
            double pitch = -ctx->moment_sp.y;
            double yaw = -ctx->moment_sp.z;

            if (thrust + fabs(pitch) + fabs(roll) + fabs(yaw) > 1) {
                thrust = 1 - fabs(pitch) - fabs(roll) - fabs(yaw);
                LOG_WRN("Thrust Saturation: %10.4f", thrust);
            }

            const float k = 1600;
            synapse_msgs_Actuators* msg = &ctx->actuators;
            msg->velocity[0] = k * (thrust + pitch - roll + yaw);
            msg->velocity[1] = k * (thrust - pitch + roll + yaw);
            msg->velocity[2] = k * (thrust + pitch + roll - yaw);
            msg->velocity[3] = k * (thrust - pitch - roll - yaw);
        }

        stamp_header(&ctx->actuators.header, k_uptime_ticks());
        ctx->actuators.header.seq++;

        // publish
        zros_pub_update(&ctx->pub_actuators);
    }

    rdd2_allocation_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_allocation_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_allocation");
    k_thread_start(tid);
    return 0;
}

static int rdd2_allocation_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_allocation, rdd2_allocation_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_allocation, &sub_rdd2_allocation, "rdd2 allocation commands", NULL);

static int rdd2_allocation_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_allocation_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
