/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

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
 
 #include "app/rdd2/casadi/bezier.h"
 #include "app/rdd2/casadi/rdd2.h"
 #include "lib/core/common/casadi/common.h"
 
 #include "input_mapping.h"
 
 #define MY_STACK_SIZE 8192
 #define MY_PRIORITY   4
 
 #ifndef M_PI
 #define M_PI 3.14159265358979323846
 #endif
 
 LOG_MODULE_REGISTER(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);
 
 static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);
 static const double thrust_trim = CONFIG_CEREBRI_RDD2_THRUST_TRIM * 1e-3;
 static const double thrust_delta = CONFIG_CEREBRI_RDD2_THRUST_DELTA * 1e-3;
 
 struct context {
	 struct zros_node node;
	 synapse_pb_Input input;
	 synapse_pb_Vector3 angular_velocity_ff, force_sp, accel_ff, moment_ff, velocity_sp,
		 position_sp;
	 synapse_pb_Quaternion attitude_sp, orientation_sp;
	 synapse_pb_BezierTrajectory bezier_trajectory;
	 synapse_pb_ClockOffset clock_offset;
	 synapse_pb_Status status;
	 synapse_pb_Status last_status;
	 synapse_pb_Odometry odometry_estimator;
	 synapse_pb_Twist cmd_vel;
	 struct zros_sub sub_bezier_trajectory_ethernet, sub_status, sub_input_ethernet,
		 sub_input_sbus, sub_odometry_estimator, sub_cmd_vel_ethernet,
		 sub_clock_offset_ethernet;
	 struct zros_pub pub_attitude_sp, pub_angular_velocity_ff, pub_force_sp, pub_accel_ff,
		 pub_moment_ff, pub_velocity_sp, pub_orientation_sp, pub_position_sp, pub_input;
	 struct k_sem running;
	 size_t stack_size;
	 k_thread_stack_t *stack_area;
	 struct k_thread thread_data;
	 double psi_sp; // yaw setpoint for input_velocity function
 };
 
 static struct context g_ctx = {
	 .node = {},
	 .input = synapse_pb_Input_init_default,
	 .attitude_sp = synapse_pb_Quaternion_init_default,
	 .angular_velocity_ff = synapse_pb_Vector3_init_default,
	 .force_sp = synapse_pb_Vector3_init_default,
	 .bezier_trajectory = synapse_pb_BezierTrajectory_init_default,
	 .status = synapse_pb_Status_init_default,
	 .last_status = synapse_pb_Status_init_default,
	 .velocity_sp = synapse_pb_Vector3_init_default,
	 .accel_ff = synapse_pb_Vector3_init_default,
	 .moment_ff = synapse_pb_Vector3_init_default,
	 .orientation_sp = synapse_pb_Quaternion_init_default,
	 .position_sp = synapse_pb_Vector3_init_default,
	 .cmd_vel = synapse_pb_Twist_init_default,
	 .sub_input_ethernet = {},
	 .sub_input_sbus = {},
	 .sub_status = {},
	 .sub_bezier_trajectory_ethernet = {},
	 .sub_clock_offset_ethernet = {},
	 .sub_odometry_estimator = {},
	 .sub_cmd_vel_ethernet = {},
	 .pub_attitude_sp = {},
	 .pub_angular_velocity_ff = {},
	 .pub_force_sp = {},
	 .pub_velocity_sp = {},
	 .pub_accel_ff = {},
	 .pub_moment_ff = {},
	 .pub_orientation_sp = {},
	 .pub_position_sp = {},
	 .pub_input = {},
	 .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	 .stack_size = MY_STACK_SIZE,
	 .stack_area = g_my_stack_area,
	 .thread_data = {},
	 .psi_sp = 0,
 };
 
 static void rdd2_command_init(struct context *ctx)
 {
	 zros_node_init(&ctx->node, "rdd2_command");
	 zros_sub_init(&ctx->sub_input_ethernet, &ctx->node, &topic_input_ethernet, &ctx->input,
			   200);
	 zros_sub_init(&ctx->sub_input_sbus, &ctx->node, &topic_input_sbus, &ctx->input, 200);
	 zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	 zros_sub_init(&ctx->sub_bezier_trajectory_ethernet, &ctx->node,
			   &topic_bezier_trajectory_ethernet, &ctx->bezier_trajectory, 10);
	 zros_sub_init(&ctx->sub_clock_offset_ethernet, &ctx->node, &topic_clock_offset_ethernet,
			   &ctx->clock_offset, 10);
	 zros_sub_init(&ctx->sub_odometry_estimator, &ctx->node, &topic_odometry_estimator,
			   &ctx->odometry_estimator, 10);
	 zros_sub_init(&ctx->sub_cmd_vel_ethernet, &ctx->node, &topic_cmd_vel_ethernet,
			   &ctx->cmd_vel, 10);
	 zros_pub_init(&ctx->pub_attitude_sp, &ctx->node, &topic_attitude_sp, &ctx->attitude_sp);
	 zros_pub_init(&ctx->pub_angular_velocity_ff, &ctx->node, &topic_angular_velocity_sp,
			   &ctx->angular_velocity_ff);
	 zros_pub_init(&ctx->pub_force_sp, &ctx->node, &topic_force_sp, &ctx->force_sp);
	 zros_pub_init(&ctx->pub_velocity_sp, &ctx->node, &topic_velocity_sp, &ctx->velocity_sp);
	 zros_pub_init(&ctx->pub_accel_ff, &ctx->node, &topic_accel_sp, &ctx->accel_ff);
	 zros_pub_init(&ctx->pub_moment_ff, &ctx->node, &topic_moment_ff, &ctx->moment_ff);
	 zros_pub_init(&ctx->pub_orientation_sp, &ctx->node, &topic_orientation_sp,
			   &ctx->orientation_sp);
	 zros_pub_init(&ctx->pub_position_sp, &ctx->node, &topic_position_sp, &ctx->position_sp);
	 zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input, &ctx->input);
	 k_sem_take(&ctx->running, K_FOREVER);
	 LOG_INF("init");
 }
 
 static void rdd2_command_fini(struct context *ctx)
 {
	 zros_sub_fini(&ctx->sub_input_ethernet);
	 zros_sub_fini(&ctx->sub_input_sbus);
	 zros_sub_fini(&ctx->sub_status);
	 zros_sub_fini(&ctx->sub_bezier_trajectory_ethernet);
	 zros_sub_fini(&ctx->sub_odometry_estimator);
	 zros_sub_fini(&ctx->sub_cmd_vel_ethernet);
	 zros_pub_fini(&ctx->pub_attitude_sp);
	 zros_pub_fini(&ctx->pub_angular_velocity_ff);
	 zros_pub_fini(&ctx->pub_velocity_sp);
	 zros_pub_fini(&ctx->pub_accel_ff);
	 zros_pub_fini(&ctx->pub_force_sp);
	 zros_pub_fini(&ctx->pub_orientation_sp);
	 zros_pub_fini(&ctx->pub_position_sp);
	 zros_node_fini(&ctx->node);
	 k_sem_give(&ctx->running);
	 LOG_INF("fini");
 }
 
 static void rdd2_command_run(void *p0, void *p1, void *p2)
 {
	 struct context *ctx = p0;
	 ARG_UNUSED(p1);
	 ARG_UNUSED(p2);
 
	 rdd2_command_init(ctx);
 
	 struct k_poll_event events[] = {
		 *zros_sub_get_event(&ctx->sub_input_ethernet),
		 *zros_sub_get_event(&ctx->sub_input_sbus),
		 *zros_sub_get_event(&ctx->sub_cmd_vel_ethernet),
		 *zros_sub_get_event(&ctx->sub_bezier_trajectory_ethernet),
	 };
 
	 double dt = 0;
	 int64_t ticks_last = k_uptime_ticks();
 
	 while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
		 // wait for input event, publish at 1 Hz regardless
		 int rc = 0;
		 rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		 if (rc != 0) {
			 // LOG_DBG("not receiving input");
			 ctx->status.mode = synapse_pb_Status_Mode_MODE_ATTITUDE;
			 for (int i = 0; i < ctx->input.channel_count; i++) {
				 ctx->input.channel[i] = 0;
			 }
			 ctx->cmd_vel.linear.x = 0;
			 ctx->cmd_vel.linear.y = 0;
			 ctx->cmd_vel.linear.z = 0;
			 ctx->cmd_vel.angular.x = 0;
			 ctx->cmd_vel.angular.y = 0;
			 ctx->cmd_vel.angular.z = 0;
			 ctx->cmd_vel.has_linear = true;
			 ctx->cmd_vel.has_angular = true;
		 }
 
		 if (zros_sub_update_available(&ctx->sub_status)) {
			 // record last status
			 ctx->last_status = ctx->status;
			 zros_sub_update(&ctx->sub_status);
		 }
 
		 // prioritize onboard sbus input
		 if (zros_sub_update_available(&ctx->sub_input_sbus)) {
			 zros_sub_update(&ctx->sub_input_sbus);
			 zros_pub_update(&ctx->pub_input);
			 ctx->status.input_source =
				 synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL;
		 } else if (zros_sub_update_available(&ctx->sub_input_ethernet)) {
			 zros_sub_update(&ctx->sub_input_ethernet);
			 zros_pub_update(&ctx->pub_input);
			 ctx->status.input_source =
				 synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET;
		 }
 
		 zros_sub_update(&ctx->sub_bezier_trajectory_ethernet);
		 zros_sub_update(&ctx->sub_odometry_estimator);
		 zros_sub_update(&ctx->sub_cmd_vel_ethernet);
 
		 // calculate dt
		 int64_t ticks_now = k_uptime_ticks();
		 dt = (double)(ticks_now - ticks_last) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		 ticks_last = ticks_now;
		 if (dt < 0 || dt > 0.5) {
			 LOG_DBG("input update rate too low");
			 continue;
		 }
 
		 // input data
		 double input_aetr[4] = {
			 ctx->input.channel[CH_RIGHT_STICK_RIGHT],
			 ctx->input.channel[CH_RIGHT_STICK_UP],
			 ctx->input.channel[CH_LEFT_STICK_UP],
			 -ctx->input.channel[CH_LEFT_STICK_RIGHT],
		 };
 
		 // estimated attitude quaternion
		 double q[4] = {ctx->odometry_estimator.pose.orientation.w,
						ctx->odometry_estimator.pose.orientation.x,
						   ctx->odometry_estimator.pose.orientation.y,
						   ctx->odometry_estimator.pose.orientation.z};
 
		 // handle joy based on mode
		 if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ATTITUDE_RATE) {
			 double omega[3];
			 double thrust;
			 {
				 // input_acro:(thrust_trim,thrust_delta,input_aetr[4])->(omega[3],thrust)
				 CASADI_FUNC_ARGS(input_acro);
 
				 args[0] = &thrust_trim;
				 args[1] = &thrust_delta;
				 args[2] = input_aetr;
 
				 res[0] = omega;
				 res[1] = &thrust;
 
				 CASADI_FUNC_CALL(input_acro);
			 }
 
			 bool data_ok = true;
			 for (int i = 0; i < 3; i++) {
				 if (!isfinite(omega[i])) {
					 LOG_ERR("omega[%d] not finite: %10.4f", i, omega[i]);
					 data_ok = false;
					 break;
				 }
			 }
			 if (!isfinite(thrust)) {
				 LOG_ERR("thrust not finite: %10.4f", thrust);
				 data_ok = false;
			 }
 
			 if (data_ok) {
				 // angular velocity set point
				 ctx->angular_velocity_ff.x = omega[0];
				 ctx->angular_velocity_ff.y = omega[1];
				 ctx->angular_velocity_ff.z = omega[2];
				 zros_pub_update(&ctx->pub_angular_velocity_ff);
 
				 // thrust pass through
				 ctx->force_sp.z = thrust;
				 zros_pub_update(&ctx->pub_force_sp);
			 }
 
		 } else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ATTITUDE) {
			 double qr[4];
			 double thrust;
			 {
				 /* input_auto_level:(thrust_trim,thrust_delta,input_aetr[4],q[4])->(q_r[4],thrust)
				  */
				 CASADI_FUNC_ARGS(input_auto_level);
 
				 args[0] = &thrust_trim;
				 args[1] = &thrust_delta;
				 args[2] = input_aetr;
				 args[3] = q;
 
				 res[0] = qr;
				 res[1] = &thrust;
 
				 CASADI_FUNC_CALL(input_auto_level);
			 }
 
			 bool data_ok = true;
			 for (int i = 0; i < 3; i++) {
				 if (!isfinite(qr[i])) {
					 LOG_ERR("qr[%d] not finite: %10.4f", i, qr[i]);
					 data_ok = false;
					 break;
				 }
			 }
 
			 if (!isfinite(thrust)) {
				 LOG_ERR("thrust not finite: %10.4f", thrust);
				 data_ok = false;
			 }
 
			 // attitude set point
			 if (data_ok) {
				 ctx->attitude_sp.w = qr[0];
				 ctx->attitude_sp.x = qr[1];
				 ctx->attitude_sp.y = qr[2];
				 ctx->attitude_sp.z = qr[3];
				 zros_pub_update(&ctx->pub_attitude_sp);
 
				 // thrust pass through
				 ctx->force_sp.z = thrust;
				 zros_pub_update(&ctx->pub_force_sp);
			 }
 
		 } else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_VELOCITY) {
 
			 bool now_vel =
				 ctx->last_status.mode != synapse_pb_Status_Mode_MODE_VELOCITY;
			 bool now_armed =
				 (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) &&
				 (ctx->last_status.arming != synapse_pb_Status_Arming_ARMING_ARMED);
 
			 // reset position setpoint and yaw if now velocity mode or now armed
			 if (now_vel || now_armed) {
				 LOG_INF("position_sp, psi_sp reset");
				 ctx->position_sp.x = ctx->odometry_estimator.pose.position.x;
				 ctx->position_sp.y = ctx->odometry_estimator.pose.position.y;
				 ctx->position_sp.z = ctx->odometry_estimator.pose.position.z;
				 
				 // Get current yaw from quaternion for reset
				 double yaw, pitch, roll;
				 {
					 CASADI_FUNC_ARGS(quat_to_eulerB321);
					 args[0] = q;
					 res[0] = &yaw;
					 res[1] = &pitch;
					 res[2] = &roll;
					 CASADI_FUNC_CALL(quat_to_eulerB321);
				 }
				 ctx->psi_sp = yaw;
			 }
 
			 double yaw_rate = 0;
			 double vt_b[3];
 
			 if (ctx->status.topic_source ==
				 synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT) {
 
				 {
					 // input_velocity:(input_aetr[4])->(vb[3],psi_vel_sp)
					 CASADI_FUNC_ARGS(input_velocity);
					 args[0] = input_aetr;
					 res[0] = vt_b;
					 res[1] = &yaw_rate;
					 CASADI_FUNC_CALL(input_velocity);
					 // LOG_INF("input velocity: %10.4f %10.4f %10.4f", vt_b[0],
					 // vt_b[1], vt_b[2]);
				 }
			 } else if (ctx->status.topic_source ==
					synapse_pb_Status_TopicSource_TOPIC_SOURCE_ETHERNET) {
				 yaw_rate = ctx->cmd_vel.angular.z;
				 vt_b[0] = ctx->cmd_vel.linear.x;
				 vt_b[1] = ctx->cmd_vel.linear.y;
				 vt_b[2] = ctx->cmd_vel.linear.z;
				 // LOG_INF("offboard yawrate: %10.4f vbx: %10.4f %10.4f %10.4f",
				 // yaw_rate, vbx, vby, vbz);
				 // LOG_INF("input velocity eth: %10.4f %10.4f %10.4f", vt_b[0],
				 // vt_b[1], vt_b[2]);
			 }
 
			 double pw[3] = {ctx->odometry_estimator.pose.position.x,
					 ctx->odometry_estimator.pose.position.y,
					 ctx->odometry_estimator.pose.position.z};
			 double pw_sp[3] = {
				 ctx->position_sp.x,
				 ctx->position_sp.y,
				 ctx->position_sp.z,
			 };
			 double vw_sp[3], aw_sp[3];
			 double reset_position = 0;
			 double q_sp[4];
			 {
				 // velocity_control:(dt,psi_sp,pw_sp[3],
				 //   pw[3],vb[3],psi_vel_sp,reset_position)
				 //   ->(psi_sp1,pw_sp1[3],vw_sp[3],aw_sp[3],q_sp[4])
				 CASADI_FUNC_ARGS(velocity_control);
 
				 args[0] = &dt;
				 args[1] = &ctx->psi_sp;
				 args[2] = pw_sp;
				 args[3] = pw;
				 args[4] = vt_b;
				 args[5] = &yaw_rate;
				 args[6] = &reset_position;
 
				 res[0] = &ctx->psi_sp;
				 res[1] = pw_sp;
				 res[2] = vw_sp;
				 res[3] = aw_sp;
				 res[4] = q_sp;
 
				 CASADI_FUNC_CALL(velocity_control);
			 }
			 
			 //LOG_INF("psi_sp1: %10.4f", psi_sp1);
 
			 // position setpoint
			 ctx->position_sp.x = pw_sp[0];
			 ctx->position_sp.y = pw_sp[1];
			 ctx->position_sp.z = pw_sp[2];
			 zros_pub_update(&ctx->pub_position_sp);
 
			 // velocity setpoint
			 ctx->velocity_sp.x = vw_sp[0];
			 ctx->velocity_sp.y = vw_sp[1];
			 ctx->velocity_sp.z = vw_sp[2];
			 zros_pub_update(&ctx->pub_velocity_sp);
 
			 // orientation setpoint (from input_velocity function)
			 ctx->orientation_sp.w = q_sp[0];
			 ctx->orientation_sp.x = q_sp[1];
			 ctx->orientation_sp.y = q_sp[2];
			 ctx->orientation_sp.z = q_sp[3];
			 zros_pub_update(&ctx->pub_orientation_sp);
 
			 // acceleration setpoint
			 ctx->accel_ff.x = aw_sp[0];
			 ctx->accel_ff.y = aw_sp[1];
			 ctx->accel_ff.z = aw_sp[2];
			 zros_pub_update(&ctx->pub_accel_ff);
 
		 } else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_BEZIER) {
			 // goal -> given position goal, find cmd_vel
			 uint64_t time_start_nsec =
				 (ctx->bezier_trajectory.time_start.seconds * 1e9 +
				  ctx->bezier_trajectory.time_start.nanos);
			 uint64_t time_stop_nsec = time_start_nsec;
 
			 // get current time
			 uint64_t time_nsec =
				 (k_uptime_get() * 1e6 + ctx->clock_offset.offset.seconds * 1e9 +
				  ctx->clock_offset.offset.nanos);
 
			 if (time_nsec < time_start_nsec) {
				 LOG_DBG("time current: %" PRIu64 " ns < time start: %" PRIu64
					 "  ns, time out of range of trajectory\n",
					 time_nsec, time_start_nsec);
				 // stop(ctx);
				 continue;
			 }
 
			 // find current trajectory index, time_start, and time_stop
			 int curve_index = 0;
 
			 while (true) {
 
				 synapse_pb_BezierTrajectory_Curve *curve =
					 &ctx->bezier_trajectory.curves[curve_index];
				 synapse_pb_BezierTrajectory_Curve *curve_prev =
					 &ctx->bezier_trajectory.curves[curve_index - 1];
 
				 uint64_t curve_stop_nsec =
					 curve->time_stop.seconds * 1e9 + curve->time_stop.nanos;
				 uint64_t curve_stop_prev_nsec =
					 curve_prev->time_stop.seconds * 1e9 +
					 curve_prev->time_stop.nanos;
 
				 // check if time handled by current trajectory
				 if (time_nsec < curve_stop_nsec) {
					 time_stop_nsec = curve_stop_nsec;
					 if (curve_index > 0) {
						 time_start_nsec = curve_stop_prev_nsec;
					 }
					 break;
				 }
 
				 // next index
				 curve_index++;
 
				 // check if index exceeds bounds
				 if (curve_index >= ctx->bezier_trajectory.curves_count) {
					 // LOG_DBG("curve index exceeds bounds");
					 // stop(ctx);
					 break;
				 }
			 }
 
			 if (curve_index < ctx->bezier_trajectory.curves_count) {
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
 
				 // bezier_multirotor:(t,T,PX[1x8],PY[1x8],PX[1x8],Ppsi[1x4])
				 // ->(x,y,z,psi,dpsi,ddpsi,V,a,j,s)
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
				 // world to body
				 // f_ref:(psi,psi_dot,psi_ddot,v_e[3],a_e[3],j_e[3],s_e[3])
				 // ->(v_b[3],quat[4],omega_eb_b[3],omega_dot_eb_b[3],M_b[3],T)
				 {
					 CASADI_FUNC_ARGS(f_ref);
 
					 args[0] = &psi;
					 args[1] = &dpsi;
					 args[2] = &ddpsi;
					 args[3] = v;
					 args[4] = a;
					 args[5] = j;
					 args[6] = s;
 
					 res[0] = q_att;
					 res[1] = omega;
					 res[2] = M;
					 
					 CASADI_FUNC_CALL(f_ref);
				 }
 
				 /* euler to quat */
				 double q_orientation[4];
				 {
					 CASADI_FUNC_ARGS(eulerB321_to_quat);
					 double phi = 0;
					 double theta = 0;
					 args[0] = &psi;
					 args[1] = &theta;
					 args[2] = &phi;
 
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
				 // ctx->attitude_sp.w = q_att[0];
				 // ctx->attitude_sp.x = q_att[1];
				 // ctx->attitude_sp.y = q_att[2];
				 // ctx->attitude_sp.z = q_att[3];
 
				 // angular velocity ff
				 ctx->angular_velocity_ff.x = omega[0];
				 ctx->angular_velocity_ff.y = omega[1];
				 ctx->angular_velocity_ff.z = omega[2];
				 zros_pub_update(&ctx->pub_angular_velocity_ff);
 
				 // moment ff
				 ctx->moment_ff.x = 0; // M[0];
				 ctx->moment_ff.y = 0; // M[1];
				 ctx->moment_ff.z = 0; // M[2];
				 zros_pub_update(&ctx->pub_moment_ff);
 
				 // orientation sp
				 ctx->orientation_sp.w = q_orientation[0];
				 ctx->orientation_sp.x = q_orientation[1];
				 ctx->orientation_sp.y = q_orientation[2];
				 ctx->orientation_sp.z = q_orientation[3];
				 zros_pub_update(&ctx->pub_orientation_sp);
			 }
 
		 } else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_UNKNOWN) {
			 // LOG_ERR("unknown mode");
		 }
	 }
 
	 rdd2_command_fini(ctx);
 }
 
 static int start(struct context *ctx)
 {
	 k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
					   rdd2_command_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	 k_thread_name_set(tid, "rdd2_command");
	 k_thread_start(tid);
	 return 0;
 }
 
 static int rdd2_command_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
 {
	 ARG_UNUSED(argc);
	 struct context *ctx = data;
 
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
 
 SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_command, rdd2_command_cmd_handler, (start, &g_ctx, "start"),
				  (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));
 
 SHELL_CMD_REGISTER(rdd2_command, &sub_rdd2_command, "rdd2 command arguments", NULL);
 
 static int rdd2_command_sys_init(void)
 {
	 return start(&g_ctx);
 };
 
 SYS_INIT(rdd2_command_sys_init, APPLICATION, 1);
 
 // vi: ts=4 sw=4 et
 