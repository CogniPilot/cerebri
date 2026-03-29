#ifndef CEREBRI_RATE_CONTROL_H_
#define CEREBRI_RATE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

#define CEREBRI_CONTROL_PERIOD_US          1250
#define CEREBRI_RC_STALE_TIMEOUT_MS        100
#define CEREBRI_THROTTLE_ARM_MAX          1050
#define CEREBRI_PID_INTEGRATE_THROTTLE_MIN 0.02f

struct cerebri_pid_axis {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prev_measurement;
	float derivative_lpf;
};

struct cerebri_rate_controller {
	struct cerebri_pid_axis roll;
	struct cerebri_pid_axis pitch;
	struct cerebri_pid_axis yaw;
};

void cerebri_rate_controller_init(struct cerebri_rate_controller *controller);
void cerebri_rate_controller_reset(struct cerebri_rate_controller *controller);
bool cerebri_rate_arm_switch_high(const cerebri_topic_RcChannels16_t *rc);
int32_t cerebri_rate_throttle_us(const cerebri_topic_RcChannels16_t *rc);
float cerebri_rate_throttle_input_from_rc(const cerebri_topic_RcChannels16_t *rc);
float cerebri_rate_throttle_command(float throttle_input, bool armed);
void cerebri_rate_desired_from_rc(const cerebri_topic_RcChannels16_t *rc,
				  cerebri_topic_RateTriplet_t *rate_desired);
void cerebri_rate_controller_step(struct cerebri_rate_controller *controller,
				  const cerebri_topic_RateTriplet_t *rate_desired,
				  const cerebri_topic_Vec3f_t *gyro, float dt,
				  bool integrate,
				  cerebri_topic_RateTriplet_t *rate_cmd);
void cerebri_mix_quad_x(float throttle, const cerebri_topic_RateTriplet_t *rate_cmd,
			cerebri_topic_MotorValues4f_t *motors);

#endif
