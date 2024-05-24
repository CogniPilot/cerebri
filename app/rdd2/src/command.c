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
#include "casadi/gen/bezier.h"

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);
static const double deg2rad = M_PI / 180.0;

struct context {
    struct zros_node node;
    synapse_msgs_Joy joy;
    synapse_msgs_Vector3 angular_velocity_ff, force_sp, accel_ff, moment_ff, velocity_sp, position_sp;
    synapse_msgs_Quaternion attitude_sp, orientation_sp;
    synapse_msgs_BezierTrajectory bezier_trajectory;
    synapse_msgs_Time clock_offset;
    synapse_msgs_Status status;
    synapse_msgs_Status last_status;
    synapse_msgs_Odometry estimator_odometry;
    struct zros_sub sub_bezier_trajectory;
    struct zros_sub sub_clock_offset;
    struct zros_sub sub_status;
    struct zros_sub sub_joy;
    struct zros_sub sub_estimator_odometry;
	// attitude_sp -> Ceb in q
    struct zros_pub pub_attitude_sp, pub_angular_velocity_ff, pub_force_sp, pub_accel_ff, pub_moment_ff,
        pub_velocity_sp, pub_orientation_sp, pub_position_sp;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    double camera_yaw;
};

static struct context g_ctx = {
    .node = {},
    .joy = synapse_msgs_Joy_init_default,
    .attitude_sp = synapse_msgs_Quaternion_init_default,
    .angular_velocity_ff = synapse_msgs_Vector3_init_default,
    .force_sp = synapse_msgs_Vector3_init_default,
    .bezier_trajectory = synapse_msgs_BezierTrajectory_init_default,
    .clock_offset = synapse_msgs_Time_init_default,
    .status = synapse_msgs_Status_init_default,
    .last_status = synapse_msgs_Status_init_default,
    .velocity_sp = synapse_msgs_Vector3_init_default,
    .accel_ff = synapse_msgs_Vector3_init_default,
    .moment_ff = synapse_msgs_Vector3_init_default,
    .orientation_sp = synapse_msgs_Quaternion_init_default,
    .position_sp = synapse_msgs_Vector3_init_default,
    .sub_joy = {},
    .sub_status = {},
    .sub_bezier_trajectory = {},
    .sub_clock_offset = {},
    .sub_estimator_odometry = {},
    .pub_attitude_sp = {},
    .pub_angular_velocity_ff = {},
    .pub_force_sp = {},
    .pub_velocity_sp = {},
    .pub_accel_ff = {},
    .pub_moment_ff = {},
    .pub_orientation_sp = {},
    .pub_position_sp = {},
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .camera_yaw = 0,
};

static void rdd2_command_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "rdd2_command");
    zros_sub_init(&ctx->sub_joy, &ctx->node, &topic_joy, &ctx->joy, 10);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_sub_init(&ctx->sub_bezier_trajectory, &ctx->node, &topic_bezier_trajectory, &ctx->bezier_trajectory, 10);
    zros_sub_init(&ctx->sub_clock_offset, &ctx->node, &topic_clock_offset, &ctx->clock_offset, 10);
	zros_sub_init(&ctx->sub_estimator_odometry, &ctx->node, &topic_estimator_odometry, &ctx->estimator_odometry, 10);
    zros_pub_init(&ctx->pub_attitude_sp, &ctx->node,
        &topic_attitude_sp, &ctx->attitude_sp);
    zros_pub_init(&ctx->pub_angular_velocity_ff, &ctx->node,
        &topic_angular_velocity_ff, &ctx->angular_velocity_ff);
    zros_pub_init(&ctx->pub_force_sp, &ctx->node,
        &topic_force_sp, &ctx->force_sp);
    zros_pub_init(&ctx->pub_velocity_sp, &ctx->node,
        &topic_velocity_sp, &ctx->velocity_sp);
    zros_pub_init(&ctx->pub_accel_ff, &ctx->node,
        &topic_accel_ff, &ctx->accel_ff);
    zros_pub_init(&ctx->pub_moment_ff, &ctx->node,
        &topic_moment_ff, &ctx->moment_ff);
    zros_pub_init(&ctx->pub_orientation_sp, &ctx->node,
        &topic_orientation_sp, &ctx->orientation_sp);
    zros_pub_init(&ctx->pub_position_sp, &ctx->node,
        &topic_position_sp, &ctx->position_sp);
    atomic_set(&ctx->running, 1);
}

static void rdd2_command_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_sub_fini(&ctx->sub_joy);
    zros_sub_fini(&ctx->sub_status);
    zros_sub_fini(&ctx->sub_bezier_trajectory);
    zros_sub_fini(&ctx->sub_estimator_odometry);
    zros_pub_fini(&ctx->pub_attitude_sp);
    zros_pub_fini(&ctx->pub_angular_velocity_ff);
    zros_pub_fini(&ctx->pub_velocity_sp);
    zros_pub_fini(&ctx->pub_accel_ff);
    zros_pub_fini(&ctx->pub_moment_ff);
    zros_pub_fini(&ctx->pub_force_sp);
    zros_pub_fini(&ctx->pub_orientation_sp);
    zros_pub_fini(&ctx->pub_position_sp);
    atomic_set(&ctx->running, 0);
}

static void rdd2_command_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rdd2_command_init(ctx);

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
            // LOG_DBG("not receiving joy");
            ctx->status.mode = synapse_msgs_Status_Mode_MODE_MANUAL;
            ctx->joy.axes[JOY_AXES_ROLL] = 0;
            ctx->joy.axes[JOY_AXES_PITCH] = 0;
            ctx->joy.axes[JOY_AXES_YAW] = 0;
            ctx->joy.axes[JOY_AXES_THRUST] = 0;
        }

        if (zros_sub_update_available(&ctx->sub_joy)) {
            zros_sub_update(&ctx->sub_joy);
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            // record last status
            ctx->last_status = ctx->status;
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_bezier_trajectory)) {
            zros_sub_update(&ctx->sub_bezier_trajectory);
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

        // joy data
        double joy_roll = (double)ctx->joy.axes[JOY_AXES_ROLL];
        double joy_pitch = (double)ctx->joy.axes[JOY_AXES_PITCH];
        double joy_yaw = (double)ctx->joy.axes[JOY_AXES_YAW];
        double joy_thrust = (double)ctx->joy.axes[JOY_AXES_THRUST];

        // estimated attitude quaternion
        double q[4];
        q[0] = ctx->estimator_odometry.pose.pose.orientation.w;
        q[1] = ctx->estimator_odometry.pose.pose.orientation.x;
        q[2] = ctx->estimator_odometry.pose.pose.orientation.y;
        q[3] = ctx->estimator_odometry.pose.pose.orientation.z;

        // handle joy based on mode
        if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_MANUAL) {

            double qr[4];
            double thrust;
            {
                /* joy_auto_level:(joy_roll,joy_pitch,joy_yaw,joy_thrust,q[4])->(qr[4],thrust) */
                CASADI_FUNC_ARGS(joy_auto_level);

                args[0] = &joy_roll;
                args[1] = &joy_pitch;
                args[2] = &joy_yaw;
                args[3] = &joy_thrust;
                args[4] = q;

                res[0] = qr;
                res[1] = &thrust;

                CASADI_FUNC_CALL(joy_auto_level);
            }

            // attitude set point
            ctx->attitude_sp.w = qr[0];
            ctx->attitude_sp.x = qr[1];
            ctx->attitude_sp.y = qr[2];
            ctx->attitude_sp.z = qr[3];
            zros_pub_update(&ctx->pub_attitude_sp);

            // thrust pass through
            ctx->force_sp.z = thrust;
            zros_pub_update(&ctx->pub_force_sp);

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_CMD_VEL) {

            bool now_cmd_vel = ctx->last_status.mode != synapse_msgs_Status_Mode_MODE_CMD_VEL;
            bool now_armed = (ctx->status.arming == synapse_msgs_Status_Arming_ARMING_ARMED) && (ctx->last_status.arming != synapse_msgs_Status_Arming_ARMING_ARMED);

            double yaw, pitch, roll;
            {
                /* quat_to_eulerB321:(q[4])->(yaw, pitch, roll) */
                CASADI_FUNC_ARGS(quat_to_eulerB321);

                args[0] = q;

                res[0] = &yaw;
                res[1] = &pitch;
                res[2] = &roll;

                CASADI_FUNC_CALL(quat_to_eulerB321);
            }

            // reset position setpoint if now cmd_vel or now armed
            if (now_cmd_vel || now_armed) {
                LOG_INF("position_sp, camera_yaw reset");
                ctx->position_sp.x = ctx->estimator_odometry.pose.pose.position.x;
                ctx->position_sp.y = ctx->estimator_odometry.pose.pose.position.y;
                ctx->position_sp.z = ctx->estimator_odometry.pose.pose.position.z;
                ctx->camera_yaw = yaw;
            }

            double yaw_rate = 60 * deg2rad * joy_yaw;
            double vbx = 2.0 * joy_pitch;
            double vby = 2.0 * joy_roll;
            double vwx = vbx * cos(yaw) - vby * sin(yaw);
            double vwy = vbx * sin(yaw) + vby * cos(yaw);
            double vwz = joy_thrust;

            // position
            ctx->position_sp.x += dt * vwx;
            ctx->position_sp.y += dt * vwy;
            ctx->position_sp.z += dt * vwz;

            double e_x = ctx->position_sp.x - ctx->estimator_odometry.pose.pose.position.x;
            double e_y = ctx->position_sp.y - ctx->estimator_odometry.pose.pose.position.y;
            double e_z = ctx->position_sp.z - ctx->estimator_odometry.pose.pose.position.z;

            double norm_e = sqrt(e_x * e_x + e_y * e_y + e_z * e_z);

            const double pos_error_max = 2.0;

            // saturate position setpoint distance from vehicle
            if (norm_e > pos_error_max) {
                ctx->position_sp.x = ctx->estimator_odometry.pose.pose.position.x + e_x * pos_error_max / norm_e;
                ctx->position_sp.y = ctx->estimator_odometry.pose.pose.position.y + e_y * pos_error_max / norm_e;
                ctx->position_sp.z = ctx->estimator_odometry.pose.pose.position.z + e_z * pos_error_max / norm_e;
            }

            // position setpoint
            zros_pub_update(&ctx->pub_position_sp);

            // velocity setpoint
            ctx->velocity_sp.x = vwx;
            ctx->velocity_sp.y = vwy;
            ctx->velocity_sp.z = vwz;
            zros_pub_update(&ctx->pub_velocity_sp);

            // desired camera direction
            ctx->camera_yaw += dt * yaw_rate;

            double qc[4];
            {
                /* eulerB321_to_quat:(yaw, pitch, roll)->(q[4]) */
                CASADI_FUNC_ARGS(eulerB321_to_quat);
                double pitch_r = 0;
                double roll_r = 0;
                args[0] = &ctx->camera_yaw;
                args[1] = &pitch_r;
                args[2] = &roll_r;
                res[0] = qc;
                CASADI_FUNC_CALL(eulerB321_to_quat);
            }

            // orientation setpoint
            ctx->orientation_sp.w = qc[0];
            ctx->orientation_sp.x = qc[1];
            ctx->orientation_sp.y = qc[2];
            ctx->orientation_sp.z = qc[3];
            zros_pub_update(&ctx->pub_orientation_sp);

            // acceleration setpoint
            ctx->accel_ff.x = 0;
            ctx->accel_ff.y = 0;
            ctx->accel_ff.z = 0;
            zros_pub_update(&ctx->pub_accel_ff);

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_AUTO) {
            // TODO bezier curve here
			// goal -> given position goal, find cmd_vel
    		uint64_t time_start_nsec = ctx->bezier_trajectory.time_start;
    		uint64_t time_stop_nsec = time_start_nsec;

    		// get current time
    		uint64_t time_nsec = k_uptime_get() * 1e6 + ctx->clock_offset.sec * 1e9 + ctx->clock_offset.nanosec;

			if (time_nsec < time_start_nsec) {
        		LOG_DBG("time current: %" PRIu64
                		" ns < time start: %" PRIu64
                		"  ns, time out of range of trajectory\n",
            		time_nsec, time_start_nsec);
        		return;
    		}
			
			// find current trajectory index, time_start, and time_stop
    		int curve_index = 0;
			while (true) {

        		// check if time handled by current trajectory
        		if (time_nsec < ctx->bezier_trajectory.curves[curve_index].time_stop) {
            		time_stop_nsec = ctx->bezier_trajectory.curves[curve_index].time_stop;
            		if (curve_index > 0) {
                		time_start_nsec = ctx->bezier_trajectory.curves[curve_index - 1].time_stop;
            		}
            		break;
        		}

        		// next index
        		curve_index++;
	
        		// check if index exceeds bounds
        		if (curve_index >= ctx->bezier_trajectory.curves_count) {
            		// LOG_ERR("curve index exceeds bounds");
            		return;
        		}
    		}

			double T = (time_stop_nsec - time_start_nsec) * 1e-9;
			double t = (time_nsec - time_start_nsec) * 1e-9;
			double x, y, z, psi, dpsi, ddpsi = 0;
			double v[3], a[3], j[3], s[3];
			double PX[8], PY[8], PZ[8], Ppsi[4];
			for (int i = 0; i < 8; i++) {
				PX[i] = ctx->bezier_trajectory.curves[curve_index].x[i];
				PY[i] = ctx->bezier_trajectory.curves[curve_index].y[i];
				PZ[i] = ctx->bezier_trajectory.curves[curve_index].z[i];
			}

			for (int i = 0; i < 4; i++) {
				Ppsi[i] = ctx->bezier_trajectory.curves[curve_index].yaw[i];
			}

			/* bezier_multirotor:(t,T,PX[1x8],PY[1x8],PX[1x8],Ppsi[1x4])->(x,y,z,psi,dpsi,ddpsi,V,a,j,s) */
    		{
        		CASADI_FUNC_ARGS(bezier_multirotor);
        		args[0] = &t;
        		args[1] = &T;
        		args[2] = PX;
        		args[3] = PY;
				args[4] = PZ;
				args[5] = Ppsi;
        		res[0] = &x;
        		res[1] = &y;
				res[2] = &z;
        		res[3] = &psi;
        		res[4] = &dpsi;
        		res[5] = &ddpsi;
        		res[6] = v;
        		res[7] = a;
        		res[8] = j;
				res[9] = s;
        		CASADI_FUNC_CALL(bezier_multirotor);
    		}
			
			double q_att[4], omega[3], M[3];
			/* world to body:(psi,dpsi,ddpsi,ve,ae,je,se)->(vb,Cbe,omegab,domegab,Mb,T)*/
			{
				CASADI_FUNC_ARGS(f_ref);
				args[0] = &psi;
				args[1] = &dpsi;
				args[2] = &ddpsi;
				args[3] = v;
				args[4] = a;
				args[5] = j;
				args[6] = s;
				res[1] = q_att;
				res[2] = omega;
				res[4] = M;
				CASADI_FUNC_CALL(f_ref);
			}

			/* euler to quat */
			double e[3], q_orientation[4];
			e[0] = psi;
			e[1] = 0;
			e[2] = 0;
			{
				CASADI_FUNC_ARGS(eulerB321_to_quat);
				args[0] = e;

				res[0] = q_orientation;
				CASADI_FUNC_CALL(eulerB321_to_quat);
			}

			// position sp
			ctx->position_sp.x = x;
			ctx->position_sp.y = y;
			ctx->position_sp.z = z;	
			zros_pub_update(&ctx->pub_position_sp);

			// velocity sp
			ctx->velocity_sp.x = v[0];
			ctx->velocity_sp.y = v[1];
			ctx->velocity_sp.z = v[2];
			zros_pub_update(&ctx->pub_velocity_sp);

			// acceleration ff
			ctx->accel_ff.x = a[0];
			ctx->accel_ff.y = a[1];
			ctx->accel_ff.z = a[2];
			zros_pub_update(&ctx->pub_accel_ff);

			// attitude sp
			ctx->attitude_sp.w = q_att[0];
			ctx->attitude_sp.x = q_att[1];
			ctx->attitude_sp.y = q_att[2];
			ctx->attitude_sp.z = q_att[3];

			// angular velocity ff
			ctx->angular_velocity_ff.x = omega[0];
			ctx->angular_velocity_ff.y = omega[1];
			ctx->angular_velocity_ff.z = omega[2];
			
			// moment ff
			ctx->moment_ff.x = M[0];
			ctx->moment_ff.y = M[0];
			ctx->moment_ff.z = M[0];
			
			// orientation sp
			ctx->orientation_sp.w = q_orientation[0];
			ctx->orientation_sp.x = q_orientation[1];
			ctx->orientation_sp.y = q_orientation[2];
			ctx->orientation_sp.z = q_orientation[3];

        } else if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_UNKNOWN) {
            // TODO: make acro mode

            double omega[3];
            double thrust;
            {
                // joy_acro:(joy_roll,joy_pitch,joy_yaw,joy_thrust)->(omega[3],thrust)
                CASADI_FUNC_ARGS(joy_acro);

                args[0] = &joy_roll;
                args[1] = &joy_pitch;
                args[2] = &joy_yaw;
                args[3] = &joy_thrust;

                res[0] = omega;
                res[1] = &thrust;

                CASADI_FUNC_CALL(joy_acro);
            }

            // angular velocity set point
            ctx->angular_velocity_ff.x = omega[0];
            ctx->angular_velocity_ff.y = omega[1];
            ctx->angular_velocity_ff.z = omega[2];
            zros_pub_update(&ctx->pub_angular_velocity_ff);

            // thrust pass through
            ctx->force_sp.z = thrust;
            zros_pub_update(&ctx->pub_force_sp);
        }
    }

    rdd2_command_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        rdd2_command_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "rdd2_command");
    k_thread_start(tid);
    return 0;
}

static int rdd2_command_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_command, rdd2_command_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_command, &sub_rdd2_command, "rdd2 command arguments", NULL);

static int rdd2_command_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(rdd2_command_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
