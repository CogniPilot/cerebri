/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <math.h>

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

#include <cerebri/core/casadi.h>

#include "casadi/gen/rdd2.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(rdd2_manual, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);
static const double deg2rad = M_PI / 180.0;
static const double thrust_trim = 0.5;
static const double thrust_delta = 0.1;

struct context {
    struct zros_node node;
    synapse_msgs_Joy joy;
    synapse_msgs_Vector3 attitude_sp, angular_velocity_sp, force_sp, velocity_sp, orientation_sp, position_sp;
    synapse_msgs_Status status;
    synapse_msgs_Status last_status;
    synapse_msgs_Odometry estimator_odometry;
    struct zros_sub sub_status;
    struct zros_sub sub_joy;
    struct zros_sub sub_estimator_odometry;
    struct zros_pub pub_attitude_sp, pub_angular_velocity_sp, pub_force_sp, pub_velocity_sp, pub_orientation_sp, pub_position_sp;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    double camera_yaw;
};

static struct context g_ctx = {
    .node = {},
    .joy = synapse_msgs_Joy_init_default,
    .attitude_sp = synapse_msgs_Vector3_init_default,
    .angular_velocity_sp = synapse_msgs_Vector3_init_default,
    .force_sp = synapse_msgs_Vector3_init_default,
    .status = synapse_msgs_Status_init_default,
    .last_status = synapse_msgs_Status_init_default,
    .velocity_sp = synapse_msgs_Vector3_init_default,
    .orientation_sp = synapse_msgs_Vector3_init_default,
    .position_sp = synapse_msgs_Vector3_init_default,
    .sub_joy = {},
    .sub_status = {},
    .sub_estimator_odometry = {},
    .pub_attitude_sp = {},
    .pub_angular_velocity_sp = {},
    .pub_force_sp = {},
    .pub_velocity_sp = {},
    .pub_orientation_sp = {},
    .pub_position_sp = {},
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .camera_yaw = 0,
};

static void rdd2_manual_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_manual");
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_estimator_odometry, &ctx->node, &topic_estimator_odometry, &ctx->estimator_odometry, 10);
    zros_pub_init(&ctx->pub_attitude_sp, &ctx->node,
        &topic_attitude_sp, &ctx->attitude_sp);
    zros_pub_init(&ctx->pub_angular_velocity_sp, &ctx->node,
        &topic_angular_velocity_sp, &ctx->angular_velocity_sp);
    zros_pub_init(&ctx->pub_force_sp, &ctx->node,
        &topic_force_sp, &ctx->force_sp);
    zros_pub_init(&ctx->pub_velocity_sp, &ctx->node,
        &topic_velocity_sp, &ctx->velocity_sp);
    zros_pub_init(&ctx->pub_orientation_sp, &ctx->node,
        &topic_orientation_sp, &ctx->orientation_sp);
    zros_pub_init(&ctx->pub_position_sp, &ctx->node,
        &topic_position_sp, &ctx->position_sp);
    atomic_set(&ctx->running, 1);
}

static void rdd2_manual_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_sub_fini(&ctx->sub_joy);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_estimator_odometry);
    zros_pub_fini(&ctx->pub_attitude_sp);
    zros_pub_fini(&ctx->pub_angular_velocity_sp);
    zros_pub_fini(&ctx->pub_velocity_sp);
    zros_pub_fini(&ctx->pub_force_sp);
    zros_pub_fini(&ctx->pub_orientation_sp);
    zros_pub_fini(&ctx->pub_position_sp);
    atomic_set(&ctx->running, 0);
}

static void rdd2_manual_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_manual_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_joy),
    };

    double dt = 0;
    int64_t ticks_last = k_uptime_ticks();

    while (atomic_get(&ctx->running)) {
        // wait for joystick input event, publish at 1 Hz regardless
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            // LOG_DBG("manual not receiving joy");
            ctx->status.mode = synapse_msgs_Status_Mode_MODE_CMD_VEL;
            ctx->joy.axes[JOY_AXES_ROLL] = 0;
            ctx->joy.axes[JOY_AXES_PITCH] = 0;
            ctx->joy.axes[JOY_AXES_YAW] = 0;
            ctx->joy.axes[JOY_AXES_THRUST] = 0;
        }

        if (zros_sub_update_available(&ctx->sub_joy)) {
            zros_sub_update(&ctx->sub_joy);
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_estimator_odometry)) {
            zros_sub_update(&ctx->sub_estimator_odometry);
        }

        // calculate dt
        int64_t ticks_now = k_uptime_ticks();
        dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        ticks_last = ticks_now;
        if (dt < 0 || dt > 0.5) {
            // LOG_WRN("joy update rate too low");
            continue;
        }

        // handle joy based on mode
        if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
            // attitude rate

            // angular velocity set point
            ctx->angular_velocity_sp.x = -60 * deg2rad * (double)ctx->joy.axes[JOY_AXES_ROLL];
            ctx->angular_velocity_sp.y = 60 * deg2rad * (double)ctx->joy.axes[JOY_AXES_PITCH];
            ctx->angular_velocity_sp.z = 60 * deg2rad * (double)ctx->joy.axes[JOY_AXES_YAW];
            zros_pub_update(&ctx->pub_angular_velocity_sp);

            // thrust pass through
            ctx->force_sp.z = (double)ctx->joy.axes[JOY_AXES_THRUST] * thrust_delta + thrust_trim;
            zros_pub_update(&ctx->pub_force_sp);

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_CMD_VEL) {
            // attitude

            /* quaternion_to_euler:(q[4])->(e[3]) */
            CASADI_FUNC_ARGS(quaternion_to_euler);
            double q[4];
            q[0] = ctx->estimator_odometry.pose.pose.orientation.w;
            q[1] = ctx->estimator_odometry.pose.pose.orientation.x;
            q[2] = ctx->estimator_odometry.pose.pose.orientation.y;
            q[3] = ctx->estimator_odometry.pose.pose.orientation.z;
            double e[3];
            args[0] = q;
            res[0] = e;
            CASADI_FUNC_CALL(quaternion_to_euler);

            // attitude set point
            ctx->attitude_sp.x = (e[0] + 20 * deg2rad * (double)ctx->joy.axes[JOY_AXES_YAW]);
            ctx->attitude_sp.y = 20 * deg2rad * (double)ctx->joy.axes[JOY_AXES_PITCH];
            ctx->attitude_sp.z = -20 * deg2rad * (double)ctx->joy.axes[JOY_AXES_ROLL];
            zros_pub_update(&ctx->pub_attitude_sp);

            // thrust pass through
            ctx->force_sp.z = (double)ctx->joy.axes[JOY_AXES_THRUST] * thrust_delta + thrust_trim;
            zros_pub_update(&ctx->pub_force_sp);

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_AUTO) {

            // reset position setpoint
            if (ctx->last_status.mode != synapse_msgs_Status_Mode_MODE_AUTO) {
                ctx->position_sp.x = ctx->estimator_odometry.pose.pose.position.x;
                ctx->position_sp.y = ctx->estimator_odometry.pose.pose.position.y;
                ctx->position_sp.z = ctx->estimator_odometry.pose.pose.position.z;
            }

            /* quaternion_to_euler:(q[4])->(e[3]) */
            CASADI_FUNC_ARGS(quaternion_to_euler);
            double q[4];
            q[0] = ctx->estimator_odometry.pose.pose.orientation.w;
            q[1] = ctx->estimator_odometry.pose.pose.orientation.x;
            q[2] = ctx->estimator_odometry.pose.pose.orientation.y;
            q[3] = ctx->estimator_odometry.pose.pose.orientation.z;
            double e[3];
            args[0] = q;
            res[0] = e;
            CASADI_FUNC_CALL(quaternion_to_euler);

            double yaw = e[0];
            double yaw_rate = 60 * deg2rad * (double)ctx->joy.axes[JOY_AXES_YAW];
            double vbx = 2.0 * (double)ctx->joy.axes[JOY_AXES_PITCH];
            double vby = 2.0 * (double)ctx->joy.axes[JOY_AXES_ROLL];
            double vwx = vbx * cos(yaw) - vby * sin(yaw);
            double vwy = vbx * sin(yaw) + vby * cos(yaw);
            double vwz = (double)ctx->joy.axes[JOY_AXES_THRUST];

            // position
            ctx->position_sp.x += dt * vwx;
            ctx->position_sp.y += dt * vwy;
            ctx->position_sp.z += dt * vwz;
            zros_pub_update(&ctx->pub_position_sp);

            // velocity
            ctx->velocity_sp.x = vwx;
            ctx->velocity_sp.y = vwy;
            ctx->velocity_sp.z = vwz;
            zros_pub_update(&ctx->pub_velocity_sp);

            // desired camera direction
            ctx->camera_yaw += dt * yaw_rate;
            ctx->orientation_sp.x = ctx->camera_yaw;
            zros_pub_update(&ctx->pub_orientation_sp);
        }

        // record last status
        ctx->last_status = ctx->status;
    }

    rdd2_manual_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_manual_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_manual");
    k_thread_start(tid);
    return 0;
}

static int rdd2_manual_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_manual, rdd2_manual_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_manual, &sub_rdd2_manual, "rdd2 manual commands", NULL);

static int rdd2_manual_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_manual_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
