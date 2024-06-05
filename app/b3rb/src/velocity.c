/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include "casadi/gen/b3rb.h"
#include "math.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/casadi.h>

#include "mixing.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_velocity, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    synapse_msgs_Twist cmd_vel;
    synapse_msgs_Status status;
    synapse_msgs_Actuators actuators;
    struct zros_sub sub_status, sub_cmd_vel;
    struct zros_pub pub_actuators;
    const double wheel_radius;
    const double wheel_base;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .cmd_vel = synapse_msgs_Twist_init_default,
    .status = synapse_msgs_Status_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .sub_status = {},
    .sub_cmd_vel = {},
    .pub_actuators = {},
    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .wheel_base = CONFIG_CEREBRI_B3RB_WHEEL_BASE_MM / 1000.0,
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void b3rb_velocity_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "b3rb_velocity");
    zros_sub_init(&ctx->sub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel, 10);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
    atomic_set(&ctx->running, 1);
}

static void b3rb_velocity_fini(struct context* ctx)
{
    LOG_INF("fini");
    atomic_set(&ctx->running, 0);
    zros_pub_fini(&ctx->pub_actuators);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_cmd_vel);
    zros_node_fini(&ctx->node);
}

// computes actuators from cmd_vel
static void b3rb_velocity_update(struct context* ctx)
{
    double turn_angle = 0;
    double omega_fwd = 0;
    double V = ctx->cmd_vel.linear.x;
    double omega = ctx->cmd_vel.angular.z;
    double delta = 0;

    CASADI_FUNC_ARGS(ackermann_steering);
    args[0] = &ctx->wheel_base;
    args[1] = &omega;
    args[2] = &V;
    res[0] = &delta;
    CASADI_FUNC_CALL(ackermann_steering);

    omega_fwd = V / ctx->wheel_radius;
    if (fabs(V) > 0.01) {
        turn_angle = delta;
    }

    bool armed = ctx->status.arming == synapse_msgs_Status_Arming_ARMING_ARMED;

    b3rb_set_actuators(&ctx->actuators, turn_angle, omega_fwd, armed);

    // publish
    zros_pub_update(&ctx->pub_actuators);
}

static void b3rb_velocity_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    b3rb_velocity_init(ctx);

    while (atomic_get(&ctx->running)) {
        struct k_poll_event events[] = {
            *zros_sub_get_event(&ctx->sub_cmd_vel),
        };
        int rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

        if (rc < 0) {
            LOG_DBG("not receiving cmd_vel");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_cmd_vel)) {
            zros_sub_update(&ctx->sub_cmd_vel);
        }

        // handle modes
        if (ctx->status.mode != synapse_msgs_Status_Mode_MODE_ACTUATORS) {
            b3rb_velocity_update(ctx);
        }
    }

    b3rb_velocity_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        b3rb_velocity_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "b3rb_velocity");
    k_thread_start(tid);
    return 0;
}

static int b3rb_velocity_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct context* ctx = data;
    if (argc != 1) {
        LOG_ERR("must have one argument");
        return -1;
    }

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

SHELL_SUBCMD_DICT_SET_CREATE(sub_b3rb_velocity, b3rb_velocity_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(b3rb_velocity, &sub_b3rb_velocity, "b3rb velocity arguments", NULL);

static int b3rb_velocity_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(b3rb_velocity_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
