/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

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

#include "mixing.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(b3rb_command, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_Joy joy;
    synapse_msgs_Actuators actuators;
    synapse_msgs_Twist cmd_vel;
    synapse_msgs_Twist offboard_cmd_vel;
    struct zros_sub sub_status, sub_offboard_joy, sub_offboard_cmd_vel;
    struct zros_pub pub_actuators, pub_cmd_vel;
    const double wheel_radius;
    const double max_turn_angle;
    const double max_velocity;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .status = synapse_msgs_Status_init_default,
    .joy = synapse_msgs_Joy_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .sub_status = {},
    .sub_offboard_joy = {},
    .sub_offboard_cmd_vel = {},
    .pub_actuators = {},
    .pub_cmd_vel = {},
    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .max_turn_angle = CONFIG_CEREBRI_B3RB_MAX_TURN_ANGLE_MRAD / 1000.0,
    .max_velocity = CONFIG_CEREBRI_B3RB_MAX_VELOCITY_MM_S / 1000.0,
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void b3rb_command_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "b3rb_command");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_offboard_joy, &ctx->node, &topic_offboard_joy, &ctx->joy, 10);
    zros_sub_init(&ctx->sub_offboard_cmd_vel, &ctx->node, &topic_offboard_cmd_vel, &ctx->offboard_cmd_vel, 10);
    zros_pub_init(&ctx->pub_actuators, &ctx->node, &topic_actuators, &ctx->actuators);
    zros_pub_init(&ctx->pub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel);
    atomic_set(&ctx->running, 1);
}

static void b3rb_command_fini(struct context* ctx)
{
    LOG_INF("fini");
    atomic_set(&ctx->running, 0);
    zros_pub_fini(&ctx->pub_cmd_vel);
    zros_pub_fini(&ctx->pub_actuators);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_offboard_joy);
    zros_sub_fini(&ctx->sub_offboard_cmd_vel);
    zros_node_fini(&ctx->node);
}

static void b3rb_command_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    b3rb_command_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_offboard_joy),
    };

    while (atomic_get(&ctx->running)) {
        // wait for joystick input event, publish at 1 Hz regardless
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));

        if (rc != 0) {
            LOG_DBG("not receiving offboard joy");
        }

        if (zros_sub_update_available(&ctx->sub_offboard_joy)) {
            zros_sub_update(&ctx->sub_offboard_joy);
        }

        if (zros_sub_update_available(&ctx->sub_offboard_cmd_vel)) {
            zros_sub_update(&ctx->sub_offboard_cmd_vel);
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_ACTUATORS) {
            double turn_angle = ctx->max_turn_angle * (double)ctx->joy.axes[JOY_AXES_RIGHT_STICK_LEFT];
            double omega_fwd = ctx->max_velocity * (double)ctx->joy.axes[JOY_AXES_LEFT_STICK_UP] / ctx->wheel_radius;

            bool armed = ctx->status.arming == synapse_msgs_Status_Arming_ARMING_ARMED;

            b3rb_set_actuators(&ctx->actuators, turn_angle, omega_fwd, armed);
            zros_pub_update(&ctx->pub_actuators);

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_VELOCITY) {
            if (ctx->status.topic_source == synapse_msgs_Status_TopicSource_TOPIC_SOURCE_JOY) {
                ctx->cmd_vel.linear.x = 0;
                ctx->cmd_vel.linear.y = 0;
                ctx->cmd_vel.linear.z = 0;
                ctx->cmd_vel.angular.x = 0;
                ctx->cmd_vel.angular.y = 0;
                ctx->cmd_vel.angular.z = 0;
                ctx->cmd_vel.has_angular = true;
                ctx->cmd_vel.has_linear = true;
            } else if (ctx->status.topic_source == synapse_msgs_Status_TopicSource_TOPIC_SOURCE_ETHERNET) {
                ctx->cmd_vel = ctx->offboard_cmd_vel;
                ctx->cmd_vel.has_angular = true;
                ctx->cmd_vel.has_linear = true;
            }
            zros_pub_update(&ctx->pub_cmd_vel);
        }
    }

    b3rb_command_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        b3rb_command_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "b3rb_command");
    k_thread_start(tid);
    return 0;
}

static int b3rb_command_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_b3rb_command, b3rb_command_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(b3rb_command, &sub_b3rb_command, "b3rb command arguments", NULL);

static int b3rb_command_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(b3rb_command_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
