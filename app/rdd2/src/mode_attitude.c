#include "command.h"

LOG_MODULE_DECLARE(rdd2_command, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

void rdd2_mode_attitude(struct context *ctx)
{
	double qr[4];
	double thrust;
	{
		/* input_auto_level:(thrust_trim,thrust_delta,input_aetr[4],q[4])->(q_r[4],thrust)
		 */
		CASADI_FUNC_ARGS(input_auto_level);

		args[0] = &ctx->thrust_trim;
		args[1] = &ctx->thrust_delta;
		args[2] = ctx->input_aetr;
		args[3] = ctx->q;

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
		stamp_msg(&ctx->attitude_sp.stamp, k_uptime_ticks());
		ctx->attitude_sp.w = qr[0];
		ctx->attitude_sp.x = qr[1];
		ctx->attitude_sp.y = qr[2];
		ctx->attitude_sp.z = qr[3];
		zros_pub_update(&ctx->pub_attitude_sp);

		// thrust pass through
		ctx->force_sp.z = thrust;
		zros_pub_update(&ctx->pub_force_sp);
	}
}
