/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_control.h"

#include "rate_control.h"

#define RC_US_CENTER            1500
#define MAX_AUTO_LEVEL_TILT_RAD 0.61086524f
#define AUTO_LEVEL_ROLL_P_GAIN  4.0f
#define AUTO_LEVEL_PITCH_P_GAIN 4.0f

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

static float rc_norm_centered(int32_t pulse_us)
{
	return clampf((float)(pulse_us - RC_US_CENTER) / 500.0f, -1.0f, 1.0f);
}

void rdd2_attitude_controller_init(struct rdd2_attitude_controller *controller)
{
	rdd2_pid_axis_init(&controller->roll);
	rdd2_pid_axis_init(&controller->pitch);

	controller->roll.kp = AUTO_LEVEL_ROLL_P_GAIN;
	controller->roll.ki = 0.0f;
	controller->roll.kd = 0.0f;
	controller->roll.i_limit = 0.0f;
	controller->roll.output_limit = RDD2_MAX_ROLL_PITCH_RATE_RAD_S;

	controller->pitch.kp = AUTO_LEVEL_PITCH_P_GAIN;
	controller->pitch.ki = 0.0f;
	controller->pitch.kd = 0.0f;
	controller->pitch.i_limit = 0.0f;
	controller->pitch.output_limit = RDD2_MAX_ROLL_PITCH_RATE_RAD_S;

	rdd2_attitude_controller_reset(controller);
}

void rdd2_attitude_controller_reset(struct rdd2_attitude_controller *controller)
{
	rdd2_pid_axis_reset(&controller->roll);
	rdd2_pid_axis_reset(&controller->pitch);
}

void rdd2_attitude_desired_from_rc(const synapse_topic_RcChannels16_t *rc,
				   const synapse_topic_AttitudeEuler_t *attitude,
				   synapse_topic_AttitudeEuler_t *attitude_desired)
{
	const int32_t *channels;

	if (rc == NULL || attitude == NULL || attitude_desired == NULL) {
		return;
	}

	channels = rdd2_topic_rc_channels_data_const(rc);
	attitude_desired->roll =
		rc_norm_centered(channels[RDD2_ROLL_CHANNEL_INDEX]) * MAX_AUTO_LEVEL_TILT_RAD;
	attitude_desired->pitch =
		rc_norm_centered(channels[RDD2_PITCH_CHANNEL_INDEX]) * MAX_AUTO_LEVEL_TILT_RAD;
	attitude_desired->yaw = attitude->yaw;
}

void rdd2_attitude_controller_step(struct rdd2_attitude_controller *controller,
				   const synapse_topic_AttitudeEuler_t *attitude,
				   const synapse_topic_AttitudeEuler_t *attitude_desired,
				   const synapse_topic_RcChannels16_t *rc, float dt,
				   synapse_topic_RateTriplet_t *rate_desired)
{
	const int32_t *channels;

	if (controller == NULL || attitude == NULL || attitude_desired == NULL || rc == NULL ||
	    rate_desired == NULL || dt <= 0.0f) {
		return;
	}

	controller->roll.setpoint = attitude_desired->roll;
	controller->roll.measurement = attitude->roll;
	rdd2_pid_axis_step(&controller->roll, dt, false);
	rate_desired->roll = clampf(controller->roll.u, -RDD2_MAX_ROLL_PITCH_RATE_RAD_S,
				    RDD2_MAX_ROLL_PITCH_RATE_RAD_S);

	controller->pitch.setpoint = attitude_desired->pitch;
	controller->pitch.measurement = attitude->pitch;
	rdd2_pid_axis_step(&controller->pitch, dt, false);
	rate_desired->pitch = clampf(controller->pitch.u, -RDD2_MAX_ROLL_PITCH_RATE_RAD_S,
				     RDD2_MAX_ROLL_PITCH_RATE_RAD_S);

	channels = rdd2_topic_rc_channels_data_const(rc);
	rate_desired->yaw =
		-rc_norm_centered(channels[RDD2_YAW_CHANNEL_INDEX]) * RDD2_MAX_YAW_RATE_RAD_S;
}
