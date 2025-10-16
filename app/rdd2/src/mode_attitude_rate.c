#include "command.h"

LOG_MODULE_DECLARE(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

void rdd2_mode_attitude_rate(struct context *ctx)
{
	double omega[3];
	double thrust;
	{
		// input_acro:(thrust_trim,thrust_delta,input_aetr[4])->(omega[3],thrust)
		CASADI_FUNC_ARGS(input_acro);

		args[0] = &ctx->thrust_trim;
		args[1] = &ctx->thrust_delta;
		args[2] = ctx->input_aetr;

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
		stamp_msg(&ctx->angular_velocity_ff.stamp, k_uptime_ticks());
		ctx->angular_velocity_ff.x = omega[0];
		ctx->angular_velocity_ff.y = omega[1];
		ctx->angular_velocity_ff.z = omega[2];
		zros_pub_update(&ctx->pub_angular_velocity_ff);

		// thrust pass through
		stamp_msg(&ctx->force_sp.stamp, k_uptime_ticks());
		ctx->force_sp.z = thrust;
		zros_pub_update(&ctx->pub_force_sp);
	}
}
