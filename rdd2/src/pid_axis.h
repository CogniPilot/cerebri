#ifndef RDD2_PID_AXIS_H_
#define RDD2_PID_AXIS_H_

#include <stdbool.h>

struct rdd2_pid_axis {
	float e_int;
	float meas_filt;
	float error;
	float setpoint;
	float measurement;
	float u;
	float kp;
	float ki;
	float kd;
	float i_limit;
	float output_limit;
	float f_cut;
};

void rdd2_pid_axis_init(struct rdd2_pid_axis *pid);
void rdd2_pid_axis_reset(struct rdd2_pid_axis *pid);
void rdd2_pid_axis_step(struct rdd2_pid_axis *pid, float dt, bool integrate);

#endif
