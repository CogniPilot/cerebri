/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

//#include "casadi/gen/rdd2_loglinear.h"
#include "casadi/gen/rdd2.h"

#include <math.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/casadi.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(rdd2_attitude, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_Odometry estimator_odometry;
    synapse_msgs_Quaternion attitude_sp;
    synapse_msgs_Vector3 position_sp, velocity_sp, angular_velocity_sp, angular_velocity_ff;
    struct zros_sub sub_status, sub_position_sp, sub_velocity_sp, sub_attitude_sp, sub_estimator_odometry, sub_angular_velocity_ff;
    struct zros_pub pub_angular_velocity_sp;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .status = synapse_msgs_Status_init_default,
    .attitude_sp = synapse_msgs_Quaternion_init_default,
    .position_sp = synapse_msgs_Vector3_init_default,
    .velocity_sp = synapse_msgs_Vector3_init_default,
    .angular_velocity_sp = synapse_msgs_Vector3_init_default,
    .angular_velocity_ff = synapse_msgs_Vector3_init_default,
    .estimator_odometry = synapse_msgs_Odometry_init_default,
    .sub_status = {},
    .sub_position_sp = {},
    .sub_velocity_sp = {},
    .sub_attitude_sp = {},
    .sub_estimator_odometry = {},
    .pub_angular_velocity_sp = {},
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void rdd2_attitude_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_attiude");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_position_sp, &ctx->node,
        &topic_position_sp, &ctx->position_sp, 50);
    zros_sub_init(&ctx->sub_velocity_sp, &ctx->node,
        &topic_velocity_sp, &ctx->velocity_sp, 50);
    zros_sub_init(&ctx->sub_attitude_sp, &ctx->node,
        &topic_attitude_sp, &ctx->attitude_sp, 50);
    zros_sub_init(&ctx->sub_estimator_odometry, &ctx->node,
        &topic_estimator_odometry, &ctx->estimator_odometry, 50);
    zros_sub_init(&ctx->sub_angular_velocity_ff, &ctx->node,
        &topic_angular_velocity_ff, &ctx->angular_velocity_ff, 50);
    zros_pub_init(&ctx->pub_angular_velocity_sp, &ctx->node, &topic_angular_velocity_sp, &ctx->angular_velocity_sp);
    k_sem_take(&ctx->running, K_FOREVER);
}

static void rdd2_attitude_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_position_sp);
    zros_sub_fini(&ctx->sub_velocity_sp);
    zros_sub_fini(&ctx->sub_attitude_sp);
    zros_sub_fini(&ctx->sub_estimator_odometry);
    zros_sub_fini(&ctx->sub_angular_velocity_ff);
    zros_pub_fini(&ctx->pub_angular_velocity_sp);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
}

static void rdd2_attitude_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_attitude_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_estimator_odometry),
    };

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("not receiving estimator_odometry");
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_estimator_odometry)) {
            zros_sub_update(&ctx->sub_estimator_odometry);
        }

        if (zros_sub_update_available(&ctx->sub_position_sp)) {
            zros_sub_update(&ctx->sub_position_sp);
        }

        if (zros_sub_update_available(&ctx->sub_velocity_sp)) {
            zros_sub_update(&ctx->sub_velocity_sp);
        }

        if (zros_sub_update_available(&ctx->sub_attitude_sp)) {
            zros_sub_update(&ctx->sub_attitude_sp);
        }

        if (zros_sub_update_available(&ctx->sub_angular_velocity_ff)) {
            zros_sub_update(&ctx->sub_angular_velocity_ff);
        }

        if (ctx->status.mode != synapse_msgs_Status_Mode_MODE_ATTITUDE_RATE) {
            double q_wb[4] = {
                ctx->estimator_odometry.pose.pose.orientation.w,
                ctx->estimator_odometry.pose.pose.orientation.x,
                ctx->estimator_odometry.pose.pose.orientation.y,
                ctx->estimator_odometry.pose.pose.orientation.z
            };

            double q_r[4] = {
                ctx->attitude_sp.w,
                ctx->attitude_sp.x,
                ctx->attitude_sp.y,
                ctx->attitude_sp.z
            };

            const double kp[3] = {
                CONFIG_CEREBRI_RDD2_ROLL_KP * 1e-3,
                CONFIG_CEREBRI_RDD2_PITCH_KP * 1e-3,
                CONFIG_CEREBRI_RDD2_YAW_KP * 1e-3,
            };

            double omega[3];

#if defined(CONFIG_CEREBRI_RDD2_LOG_LINEAR_ATTITUDE)
            {
                double zeta[9];
                double p_w[3] = {
                    ctx->estimator_odometry.pose.pose.position.x,
                    ctx->estimator_odometry.pose.pose.position.y,
                    ctx->estimator_odometry.pose.pose.position.z
                };

                double v_b[3] = {
                    ctx->estimator_odometry.twist.twist.linear.x,
                    ctx->estimator_odometry.twist.twist.linear.y,
                    ctx->estimator_odometry.twist.twist.linear.z
                };

                double p_rw[3] = {
                    ctx->position_sp.x,
                    ctx->position_sp.y,
                    ctx->position_sp.z
                };

                double v_rw[3] = {
                    ctx->velocity_sp.x,
                    ctx->velocity_sp.y,
                    ctx->velocity_sp.z
                };

                // se23_error:(p_w[3],v_b[3],q_wb[4],p_rw[3],v_rw[3],q_r[4])->(zeta[9])

                CASADI_FUNC_ARGS(se23_error);
                args[0] = p_w;
                args[1] = v_b;
                args[2] = q_wb;
                args[3] = p_rw;
                args[4] = v_rw;
                args[5] = q_r;
                res[0] = zeta;
                CASADI_FUNC_CALL(se23_error);

                // se23_attitude_control:(kp[3],zeta[9])->(omega[3])
                CASADI_FUNC_ARGS(se23_attitude_control);
                args[0] = kp;
                args[1] = zeta;
                res[0] = omega;
                CASADI_FUNC_CALL(se23_attitude_control);
            }
#else
            {
                // attitude_control:(kp[3],q[4],q_r[4])->(omega[3])
                CASADI_FUNC_ARGS(attitude_control);
                args[0] = kp;
                args[1] = q_wb;
                args[2] = q_r;
                res[0] = omega;
                CASADI_FUNC_CALL(attitude_control);
            }
#endif

            // publish
            bool data_ok = true;
            for (int i = 0; i < 3; i++) {
                if (!isfinite(omega[i])) {
                    LOG_ERR("omega[0] not finite: %10.4f", omega[i]);
                    data_ok = false;
                }
            }

            if (data_ok) {
                ctx->angular_velocity_sp.x = omega[0] + ctx->angular_velocity_ff.x;
                ctx->angular_velocity_sp.y = omega[1] + ctx->angular_velocity_ff.y;
                ctx->angular_velocity_sp.z = omega[2] + ctx->angular_velocity_ff.z;
                zros_pub_update(&ctx->pub_angular_velocity_sp);
            }
        }
    }

    rdd2_attitude_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_attitude_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_attitude");
    k_thread_start(tid);
    return 0;
}

static int rdd2_attitude_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_attitude, rdd2_attitude_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_attitude, &sub_rdd2_attitude, "rdd2 attitude commands", NULL);

static int rdd2_attitude_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_attitude_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
