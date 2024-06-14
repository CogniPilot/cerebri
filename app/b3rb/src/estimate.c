/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdio.h>
#include <time.h>

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

#include "casadi/gen/b3rb.h"

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(b3rb_estimate, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
    struct zros_node node;
    synapse_msgs_WheelOdometry wheel_odometry;
    synapse_msgs_Imu imu;
    synapse_msgs_Odometry odometry;
    struct zros_sub sub_wheel_odometry, sub_imu;
    struct zros_pub pub_odometry;
    double x[3];
    const double wheel_radius;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

// private initialization
static struct context g_ctx = {
    .node = {},
    .wheel_odometry = synapse_msgs_WheelOdometry_init_default,
    .imu = synapse_msgs_Imu_init_default,
    .odometry = {
        .child_frame_id = "base_link",
        .has_header = true,
        .header.frame_id = "odom",
        .has_pose = true,
        .pose.has_pose = true,
        .pose.pose.has_position = true,
        .pose.pose.has_orientation = true,
    },
    .sub_wheel_odometry = {},
    .sub_imu = {},
    .pub_odometry = {},
    .x = {},
    .wheel_radius = CONFIG_CEREBRI_B3RB_WHEEL_RADIUS_MM / 1000.0,
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void b3rb_estimate_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "b3rb_estimate");
    zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 10);
    zros_sub_init(&ctx->sub_wheel_odometry, &ctx->node, &topic_wheel_odometry,
        &ctx->wheel_odometry, 10);
    zros_pub_init(&ctx->pub_odometry, &ctx->node, &topic_odometry_estimator, &ctx->odometry);
    k_sem_take(&ctx->running, K_FOREVER);
}

static void b3rb_estimate_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_pub_fini(&ctx->pub_odometry);
    zros_sub_fini(&ctx->sub_wheel_odometry);
    zros_sub_fini(&ctx->sub_imu);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
}

static bool all_finite(double* src, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (!isfinite(src[i])) {
            return false;
        }
    }
    return true;
}

static void handle_update(struct context* ctx, double* x1)
{
    bool x1_finite = all_finite(x1, ARRAY_SIZE(ctx->x));

    if (!x1_finite) {
        LOG_WRN("x1 update not finite");
    }

    if (x1_finite) {
        memcpy(ctx->x, x1, sizeof(ctx->x));
    }
}

static void b3rb_estimate_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int rc = 0;

    // LOG_DBG("started");
    b3rb_estimate_init(ctx);

    // variables
    int32_t seq = 0;
    double rotation_last = 0;

    struct k_poll_event events[1] = {};

    // wait for imu
    LOG_DBG("waiting for imu");
    events[0] = *zros_sub_get_event(&ctx->sub_imu);
    rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
    if (rc != 0) {
        LOG_DBG("did not receive imu");
        return;
    }
    if (zros_sub_update_available(&ctx->sub_imu)) {
        zros_sub_update(&ctx->sub_imu);
    }

    // wait for wheel odometry
    LOG_DBG("waiting for wheel odometry");
    events[0] = *zros_sub_get_event(&ctx->sub_wheel_odometry);
    rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
    if (rc != 0) {
        LOG_DBG("did not receive wheel odometry");
        return;
    }
    if (zros_sub_update_available(&ctx->sub_wheel_odometry)) {
        zros_sub_update(&ctx->sub_wheel_odometry);
    }

    double dt = 0;
    int64_t ticks_last = k_uptime_ticks();

    // poll on imu
    events[0] = *zros_sub_get_event(&ctx->sub_imu);

    // estimator state
    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

        // poll for imu
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("not receiving imu");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_imu)) {
            zros_sub_update(&ctx->sub_imu);
        }

        if (zros_sub_update_available(&ctx->sub_wheel_odometry)) {
            zros_sub_update(&ctx->sub_wheel_odometry);
        }

        // calculate dt
        int64_t ticks_now = k_uptime_ticks();
        dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        ticks_last = ticks_now;
        if (dt < 0 || dt > 0.5) {
            LOG_WRN("imu update rate too low");
            continue;
        }

        // get data
        double rotation = ctx->wheel_odometry.rotation;

        // negative sign due to current gearing, should be in driver
        double u = (rotation - rotation_last) * ctx->wheel_radius;
        rotation_last = rotation;

        double omega = ctx->imu.angular_velocity.z;
        // LOG_DBG("imu omega z: %10.4f", omega);

        /* predict:(x0[3],omega,u)->(x1[3]) */
        {
            double delta_theta = omega * dt;
            double x1[3];

            // LOG_DBG("predict");
            CASADI_FUNC_ARGS(predict);
            args[0] = ctx->x;
            args[1] = &delta_theta;
            args[2] = &u;
            res[0] = x1;
            CASADI_FUNC_CALL(predict);

            // update x, W
            handle_update(ctx, x1);
        }

        // publish odometry
        {
            stamp_header(&ctx->odometry.header, k_uptime_ticks());
            ctx->odometry.header.seq = seq++;

            double theta = ctx->x[2];
            ctx->odometry.pose.pose.position.x = ctx->x[0];
            ctx->odometry.pose.pose.position.y = ctx->x[1];
            ctx->odometry.pose.pose.position.z = 0;
            ctx->odometry.pose.pose.orientation.x = 0;
            ctx->odometry.pose.pose.orientation.y = 0;
            ctx->odometry.pose.pose.orientation.z = sin(theta / 2);
            ctx->odometry.pose.pose.orientation.w = cos(theta / 2);
            ctx->odometry.twist.twist.angular.z = omega;
            ctx->odometry.twist.twist.linear.x = u;
            zros_pub_update(&ctx->pub_odometry);
        }
    }

    b3rb_estimate_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        b3rb_estimate_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "b3rb_estimate");
    k_thread_start(tid);
    return 0;
}

static int b3rb_estimate_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_b3rb_estimate, b3rb_estimate_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(b3rb_estimate, &sub_b3rb_estimate, "b3rb estimate arguments", NULL);

static int b3rb_estimate_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(b3rb_estimate_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
