/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pid_axis.h"

#include <math.h>
#include <stddef.h>

#define RDD2_PID_DEFAULT_KP           0.12f
#define RDD2_PID_DEFAULT_KI           0.35f
#define RDD2_PID_DEFAULT_KD           0.0015f
#define RDD2_PID_DEFAULT_I_LIMIT      0.2f
#define RDD2_PID_DEFAULT_OUTPUT_LIMIT 0.35f
#define RDD2_PID_DEFAULT_F_CUT        25.0f
#define RDD2_PID_FILTER_OMEGA_SCALE   6.28318530717958647693f
#define RDD2_PID_UNWIND_TAU_S         0.001f

static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

void rdd2_pid_axis_init(struct rdd2_pid_axis *pid)
{
	if (pid == NULL) {
		return;
	}

	pid->kp = RDD2_PID_DEFAULT_KP;
	pid->ki = RDD2_PID_DEFAULT_KI;
	pid->kd = RDD2_PID_DEFAULT_KD;
	pid->i_limit = RDD2_PID_DEFAULT_I_LIMIT;
	pid->output_limit = RDD2_PID_DEFAULT_OUTPUT_LIMIT;
	pid->f_cut = RDD2_PID_DEFAULT_F_CUT;
	rdd2_pid_axis_reset(pid);
}

void rdd2_pid_axis_reset(struct rdd2_pid_axis *pid)
{
	if (pid == NULL) {
		return;
	}

	pid->e_int = 0.0f;
	pid->meas_filt = NAN;
	pid->error = 0.0f;
	pid->setpoint = 0.0f;
	pid->measurement = 0.0f;
	pid->u = 0.0f;
}

void rdd2_pid_axis_step(struct rdd2_pid_axis *pid, float dt, bool integrate)
{
	float derivative = 0.0f;

	if (pid == NULL || dt <= 0.0f) {
		return;
	}

	if (pid->f_cut > 0.0f) {
		float tau = 1.0f / (RDD2_PID_FILTER_OMEGA_SCALE * pid->f_cut);

		if (isnan(pid->meas_filt)) {
			pid->meas_filt = pid->measurement;
		} else {
			derivative = (pid->measurement - pid->meas_filt) / tau;
			pid->meas_filt += dt * derivative;
		}
	} else {
		pid->meas_filt = pid->measurement;
	}

	pid->error = pid->setpoint - pid->measurement;

	if (integrate) {
		pid->e_int += pid->ki * pid->error * dt;
		pid->e_int = clampf(pid->e_int, -pid->i_limit, pid->i_limit);
	} else if (pid->e_int != 0.0f) {
		float unwind_scale = 1.0f - (dt / RDD2_PID_UNWIND_TAU_S);

		if (unwind_scale < 0.0f) {
			unwind_scale = 0.0f;
		}
		pid->e_int *= unwind_scale;
	}

	pid->u = clampf((pid->kp * pid->error) + pid->e_int - (pid->kd * derivative),
			-pid->output_limit, pid->output_limit);
}
