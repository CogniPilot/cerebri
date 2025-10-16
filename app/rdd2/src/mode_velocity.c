#include "command.h"

LOG_MODULE_DECLARE(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

void rdd2_mode_velocity(struct context *ctx)
{
	bool now_vel = ctx->last_status.mode != synapse_pb_Status_Mode_MODE_VELOCITY;
	bool now_armed = (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) &&
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
			args[0] = ctx->q;
			res[0] = &yaw;
			res[1] = &pitch;
			res[2] = &roll;
			CASADI_FUNC_CALL(quat_to_eulerB321);
		}
		ctx->psi_sp = yaw;
	}

	double yaw_rate = 0;
	double vt_b[3];

	if (ctx->status.topic_source == synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT) {

		{
			// input_velocity:(input_aetr[4])->(vb[3],psi_vel_sp)
			CASADI_FUNC_ARGS(input_velocity);
			args[0] = ctx->input_aetr;
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

		args[0] = &ctx->dt;
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

	// LOG_INF("psi_sp1: %10.4f", psi_sp1);

	// position setpoint
	stamp_msg(&ctx->position_sp.stamp, k_uptime_ticks());
	ctx->position_sp.has_stamp = true;
	ctx->position_sp.x = pw_sp[0];
	ctx->position_sp.y = pw_sp[1];
	ctx->position_sp.z = pw_sp[2];
	zros_pub_update(&ctx->pub_position_sp);

	// velocity setpoint
	stamp_msg(&ctx->velocity_sp.stamp, k_uptime_ticks());
	ctx->velocity_sp.has_stamp = true;
	ctx->velocity_sp.x = vw_sp[0];
	ctx->velocity_sp.y = vw_sp[1];
	ctx->velocity_sp.z = vw_sp[2];
	zros_pub_update(&ctx->pub_velocity_sp);

	// orientation setpoint (from input_velocity function)
	stamp_msg(&ctx->orientation_sp.stamp, k_uptime_ticks());
	ctx->orientation_sp.has_stamp = true;
	ctx->orientation_sp.w = q_sp[0];
	ctx->orientation_sp.x = q_sp[1];
	ctx->orientation_sp.y = q_sp[2];
	ctx->orientation_sp.z = q_sp[3];
	zros_pub_update(&ctx->pub_orientation_sp);

	// acceleration setpoint
	stamp_msg(&ctx->accel_sp.stamp, k_uptime_ticks());
	ctx->accel_sp.has_stamp = true;
	ctx->accel_sp.x = aw_sp[0];
	ctx->accel_sp.y = aw_sp[1];
	ctx->accel_sp.z = aw_sp[2];
	zros_pub_update(&ctx->pub_accel_sp);
}
