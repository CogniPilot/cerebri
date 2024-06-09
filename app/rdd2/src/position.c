/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "casadi/gen/rdd2.h"
#include "casadi/gen/rdd2_loglinear.h"

#include <cerebri/core/casadi.h>

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(rdd2_position, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

static const double thrust_trim = CONFIG_CEREBRI_RDD2_THRUST_TRIM * 1e-3;

struct context {
    struct zros_node node;
    synapse_msgs_Status status;
    synapse_msgs_Odometry estimator_odometry;
    synapse_msgs_Vector3 force_sp, position_sp, velocity_sp, accel_ff;
    synapse_msgs_Quaternion attitude_sp, orientation_sp;
    struct zros_sub sub_status,
        sub_position_sp, sub_velocity_sp, sub_accel_ff, sub_estimator_odometry,
        sub_orientation_sp;
    struct zros_pub pub_force_sp, pub_attitude_sp;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .status = synapse_msgs_Status_init_default,
    .force_sp = synapse_msgs_Vector3_init_default,
    .estimator_odometry = synapse_msgs_Odometry_init_default,
    .attitude_sp = synapse_msgs_Quaternion_init_default,
    .orientation_sp = synapse_msgs_Quaternion_init_default,
    .position_sp = synapse_msgs_Vector3_init_default,
    .velocity_sp = synapse_msgs_Vector3_init_default,
    .accel_ff = synapse_msgs_Vector3_init_default,
    .sub_status = {},
    .sub_position_sp = {},
    .sub_velocity_sp = {},
    .sub_accel_ff = {},
    .sub_estimator_odometry = {},
    .sub_orientation_sp = {},
    .pub_force_sp = {},
    .pub_attitude_sp = {},
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .running = ATOMIC_INIT(0),
};

static void rdd2_position_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_position");
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_position_sp, &ctx->node, &topic_position_sp, &ctx->position_sp, 10);
    zros_sub_init(&ctx->sub_velocity_sp, &ctx->node, &topic_velocity_sp, &ctx->velocity_sp, 10);
    zros_sub_init(&ctx->sub_accel_ff, &ctx->node, &topic_accel_ff, &ctx->accel_ff, 10);
    zros_sub_init(&ctx->sub_estimator_odometry, &ctx->node, &topic_estimator_odometry, &ctx->estimator_odometry, 10);
    zros_sub_init(&ctx->sub_orientation_sp, &ctx->node, &topic_orientation_sp, &ctx->orientation_sp, 10);
    zros_pub_init(&ctx->pub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp);
    zros_pub_init(&ctx->pub_attitude_sp, &ctx->node, &topic_attitude_sp, &ctx->attitude_sp);
    atomic_set(&ctx->running, 1);
}

static void rdd2_position_fini(struct context* ctx)
{
    LOG_INF("fini");
    atomic_set(&ctx->running, 0);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_position_sp);
    zros_sub_fini(&ctx->sub_velocity_sp);
    zros_sub_fini(&ctx->sub_accel_ff);
    zros_sub_fini(&ctx->sub_estimator_odometry);
    zros_sub_fini(&ctx->sub_orientation_sp);
    zros_pub_fini(&ctx->pub_force_sp);
    zros_pub_fini(&ctx->pub_attitude_sp);
    zros_node_fini(&ctx->node);
}

static void rdd2_position_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_position_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_estimator_odometry),
    };

    double dt = 0;
    int64_t ticks_last = k_uptime_ticks();
    double z_i = 0; // altitude error integral

    while (atomic_get(&ctx->running)) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("pos not receiving  pose");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_position_sp)) {
            zros_sub_update(&ctx->sub_position_sp);
        }

        if (zros_sub_update_available(&ctx->sub_velocity_sp)) {
            zros_sub_update(&ctx->sub_velocity_sp);
        }

        if (zros_sub_update_available(&ctx->sub_accel_ff)) {
            zros_sub_update(&ctx->sub_accel_ff);
        }

        if (zros_sub_update_available(&ctx->sub_orientation_sp)) {
            zros_sub_update(&ctx->sub_orientation_sp);
        }

        if (zros_sub_update_available(&ctx->sub_estimator_odometry)) {
            zros_sub_update(&ctx->sub_estimator_odometry);
        }

        // calculate dt
        int64_t ticks_now = k_uptime_ticks();
        dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        ticks_last = ticks_now;
        if (dt < 0 || dt > 0.5) {
            LOG_WRN("position update rate too low");
            continue;
        }

        if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_POSITION || ctx->status.mode == synapse_msgs_Status_Mode_MODE_VELOCITY || ctx->status.mode == synapse_msgs_Status_Mode_MODE_ACCELERATION || ctx->status.mode == synapse_msgs_Status_Mode_MODE_BEZIER) {
            double zeta[9];
            {
                /* se23_error:(p_w[3],v_b[3],q_wb[4],p_rw[3],v_rw[3],q_r[4])->(zeta[9])*/
                CASADI_FUNC_ARGS(se23_error);
                double p_w[3], v_b[3], q_wb[4];
                double p_rw[3], v_rw[3], q_r[4];

                p_w[0] = ctx->estimator_odometry.pose.pose.position.x;
                p_w[1] = ctx->estimator_odometry.pose.pose.position.y;
                p_w[2] = ctx->estimator_odometry.pose.pose.position.z;
                v_b[0] = ctx->estimator_odometry.twist.twist.linear.x;
                v_b[1] = ctx->estimator_odometry.twist.twist.linear.y;
                v_b[2] = ctx->estimator_odometry.twist.twist.linear.z;
                q_wb[0] = ctx->estimator_odometry.pose.pose.orientation.w;
                q_wb[1] = ctx->estimator_odometry.pose.pose.orientation.x;
                q_wb[2] = ctx->estimator_odometry.pose.pose.orientation.y;
                q_wb[3] = ctx->estimator_odometry.pose.pose.orientation.z;

                p_rw[0] = ctx->position_sp.x;
                p_rw[1] = ctx->position_sp.y;
                p_rw[2] = ctx->position_sp.z;
                v_rw[0] = ctx->velocity_sp.x;
                v_rw[1] = ctx->velocity_sp.y;
                v_rw[2] = ctx->velocity_sp.z;
                q_r[0] = ctx->attitude_sp.w;
                q_r[1] = ctx->attitude_sp.x;
                q_r[2] = ctx->attitude_sp.y;
                q_r[3] = ctx->attitude_sp.z;

                args[0] = p_w;
                args[1] = v_b;
                args[2] = q_wb;
                args[3] = p_rw;
                args[4] = v_rw;
                args[5] = q_r;
                res[0] = zeta;
                CASADI_FUNC_CALL(se23_error);
            }
            double nT; // normalized magnitude of thrust (ratio of twice weight)
            double qr_wb[4];
            {
                const double kp[3] = {
                    CONFIG_CEREBRI_RDD2_ROLL_KP * 1e-3,
                    CONFIG_CEREBRI_RDD2_PITCH_KP * 1e-3,
                    CONFIG_CEREBRI_RDD2_YAW_KP * 1e-3,
                };
                CASADI_FUNC_ARGS(se23_position_control)
                // LOG_INF("vt_w: %10.4f %10.4f %10.4f", ctx->velocity_sp.x, ctx->velocity_sp.y, ctx->velocity_sp.z);
                double at_w[3] = {
                    ctx->accel_ff.x,
                    ctx->accel_ff.y,
                    ctx->accel_ff.z
                };
                double qc_wb[4] = {
                    ctx->orientation_sp.w,
                    ctx->orientation_sp.x,
                    ctx->orientation_sp.y,
                    ctx->orientation_sp.z
                };
                double q_wb[4] = {
                    ctx->estimator_odometry.pose.pose.orientation.w,
                    ctx->estimator_odometry.pose.pose.orientation.x,
                    ctx->estimator_odometry.pose.pose.orientation.y,
                    ctx->estimator_odometry.pose.pose.orientation.z
                };

                /* position_control:(thrust_trim,pt_w[3],vt_w[3],at_w[3],qc_wb[4],
                 * p_w[3],v_b[3],q_wb[4],z_i,dt)->(nT,qr_wb[4],z_i_2) */
                args[0] = &thrust_trim;
                args[1] = kp;
                args[2] = zeta;
                args[3] = at_w;
                args[4] = qc_wb;
                args[5] = q_wb;
                args[6] = &z_i;
                args[7] = &dt;
                res[0] = &nT;
                res[1] = qr_wb;
                res[2] = &z_i;

                CASADI_FUNC_CALL(se23_position_control)
                // LOG_INF("z_i: %10.4f", z_i);
            }

            bool data_ok = true;
            for (int i = 0; i < 4; i++) {
                if (!isfinite(qr_wb[i])) {
                    LOG_ERR("qr_wb[%d] not finite: %10.4f", i, qr_wb[i]);
                    data_ok = false;
                    break;
                }
            }

            if (!isfinite(nT)) {
                LOG_ERR("nT not finite: %10.4f", nT);
                data_ok = false;
            }

            if (data_ok) {
                ctx->attitude_sp.w = qr_wb[0];
                ctx->attitude_sp.x = qr_wb[1];
                ctx->attitude_sp.y = qr_wb[2];
                ctx->attitude_sp.z = qr_wb[3];
                zros_pub_update(&ctx->pub_attitude_sp);

                ctx->force_sp.z = nT;
                zros_pub_update(&ctx->pub_force_sp);
            }
        }
    }
    rdd2_position_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_position_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_position");
    k_thread_start(tid);
    return 0;
}

static int rdd2_position_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_position, rdd2_position_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_position, &sub_rdd2_position, "rdd2 position commands", NULL);

static int rdd2_position_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_position_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
