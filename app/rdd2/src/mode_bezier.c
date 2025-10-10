#include "command.h"

LOG_MODULE_DECLARE(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

void rdd2_mode_bezier(struct context *ctx)
{
	// goal -> given position goal, find cmd_vel
	uint64_t time_start_nsec = (ctx->bezier_trajectory.time_start.seconds * 1e9 +
				    ctx->bezier_trajectory.time_start.nanos);
	uint64_t time_stop_nsec = time_start_nsec;

	// get current time
	uint64_t time_nsec = (k_uptime_get() * 1e6 + ctx->clock_offset.offset.seconds * 1e9 +
			      ctx->clock_offset.offset.nanos);

	if (time_nsec < time_start_nsec) {
		LOG_WRN("time current: %" PRIu64 " ns < time start: %" PRIu64
			"  ns, time out of range of trajectory\n",
			time_nsec, time_start_nsec);
		return;
	}

	// find current trajectory index, time_start, and time_stop
	int curve_index = 0;

	while (true) {

		synapse_pb_BezierTrajectory_Curve *curve =
			&ctx->bezier_trajectory.curves[curve_index];
		synapse_pb_BezierTrajectory_Curve *curve_prev =
			&ctx->bezier_trajectory.curves[curve_index - 1];

		uint64_t curve_stop_nsec = curve->time_stop.seconds * 1e9 + curve->time_stop.nanos;
		uint64_t curve_stop_prev_nsec =
			curve_prev->time_stop.seconds * 1e9 + curve_prev->time_stop.nanos;

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

		double v_b[3], q_att[4], omega[3], omega_dot[3], M[3], Thrust;
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

			res[0] = v_b;
			res[1] = q_att;
			res[2] = omega;
			res[3] = omega_dot;
			res[4] = M;
			res[5] = &Thrust;

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
		ctx->moment_ff.x = M[0];
		ctx->moment_ff.y = M[1];
		ctx->moment_ff.z = M[2];
		zros_pub_update(&ctx->pub_moment_ff);

		// orientation sp
		ctx->orientation_sp.w = q_orientation[0];
		ctx->orientation_sp.x = q_orientation[1];
		ctx->orientation_sp.y = q_orientation[2];
		ctx->orientation_sp.z = q_orientation[3];
		zros_pub_update(&ctx->pub_orientation_sp);
	}
}
