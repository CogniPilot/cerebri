/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rate_control.h"

#include "motor_output.h"

#define RC_US_CENTER            1500
#define RC_US_MIN               1000
#define RC_US_MAX               2000
#define ARM_SWITCH_THRESHOLD_US 1600

struct motor_mix {
	float roll;
	float pitch;
	float yaw;
};

/*
 * Default mixer uses FLU body axes and assumes DSHOT channels 1..4 are:
 * front-right, rear-right, rear-left, front-left.
 *
 * Positive roll is about body +x (forward), so left motors increase and
 * right motors decrease.
 *
 * Positive pitch is about body +y (left), so rear motors increase and front
 * motors decrease.
 *
 * Positive yaw is about body +z (up), so yaw mix signs invert relative to the
 * previous FRD convention.
 */
static const struct motor_mix g_motor_mix[4] = {
	{.roll = -1.0f, .pitch = -1.0f, .yaw = -1.0f},
	{.roll = -1.0f, .pitch = 1.0f, .yaw = 1.0f},
	{.roll = 1.0f, .pitch = 1.0f, .yaw = -1.0f},
	{.roll = 1.0f, .pitch = -1.0f, .yaw = 1.0f},
};

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

static float rc_norm_throttle(int32_t pulse_us)
{
	return clampf((float)(pulse_us - RC_US_MIN) / (float)(RC_US_MAX - RC_US_MIN), 0.0f, 1.0f);
}

void rdd2_rate_controller_init(struct rdd2_rate_controller *controller)
{
	rdd2_pid_axis_init(&controller->roll);
	rdd2_pid_axis_init(&controller->pitch);
	rdd2_pid_axis_init(&controller->yaw);
	controller->yaw.kp = 0.20f;
	controller->yaw.ki = 0.20f;
	controller->yaw.kd = 0.0f;
	rdd2_rate_controller_reset(controller);
}

void rdd2_rate_controller_reset(struct rdd2_rate_controller *controller)
{
	rdd2_pid_axis_reset(&controller->roll);
	rdd2_pid_axis_reset(&controller->pitch);
	rdd2_pid_axis_reset(&controller->yaw);
}

bool rdd2_rate_arm_switch_high(const synapse_topic_RcChannels16_t *rc)
{
	return rdd2_topic_rc_channels_data_const(rc)[RDD2_ARM_CHANNEL_INDEX] >
	       ARM_SWITCH_THRESHOLD_US;
}

int32_t rdd2_rate_throttle_us(const synapse_topic_RcChannels16_t *rc)
{
	return rdd2_topic_rc_channels_data_const(rc)[RDD2_THROTTLE_CHANNEL_INDEX];
}

float rdd2_rate_throttle_input_from_rc(const synapse_topic_RcChannels16_t *rc)
{
	return rc_norm_throttle(rdd2_rate_throttle_us(rc));
}

float rdd2_rate_throttle_command(float throttle_input, bool armed)
{
	if (!armed) {
		return 0.0f;
	}

	return RDD2_MOTOR_IDLE_THROTTLE + (throttle_input * (1.0f - RDD2_MOTOR_IDLE_THROTTLE));
}

float rdd2_rate_yaw_desired_from_rc(const synapse_topic_RcChannels16_t *rc)
{
	return -rc_norm_centered(rdd2_topic_rc_channels_data_const(rc)[RDD2_YAW_CHANNEL_INDEX]) *
	       RDD2_MAX_YAW_RATE_RAD_S;
}

void rdd2_rate_desired_from_rc(const synapse_topic_RcChannels16_t *rc,
			       synapse_topic_RateTriplet_t *rate_desired)
{
	const int32_t *channels = rdd2_topic_rc_channels_data_const(rc);

	rate_desired->roll = rc_norm_centered(channels[RDD2_ROLL_CHANNEL_INDEX]) *
			     RDD2_MAX_ROLL_PITCH_RATE_RAD_S;
	rate_desired->pitch = rc_norm_centered(channels[RDD2_PITCH_CHANNEL_INDEX]) *
			      RDD2_MAX_ROLL_PITCH_RATE_RAD_S;
	rate_desired->yaw = rdd2_rate_yaw_desired_from_rc(rc);
}

void rdd2_rate_controller_step(struct rdd2_rate_controller *controller,
			       const synapse_topic_RateTriplet_t *rate_desired,
			       const synapse_topic_Vec3f_t *gyro, float dt, bool integrate,
			       synapse_topic_RateTriplet_t *rate_cmd)
{
	if (dt <= 0.0f) {
		*rate_cmd = (synapse_topic_RateTriplet_t){0};
		return;
	}

	controller->roll.setpoint = rate_desired->roll;
	controller->roll.measurement = gyro->x;
	rdd2_pid_axis_step(&controller->roll, dt, integrate);
	rate_cmd->roll = controller->roll.u;

	controller->pitch.setpoint = rate_desired->pitch;
	controller->pitch.measurement = gyro->y;
	rdd2_pid_axis_step(&controller->pitch, dt, integrate);
	rate_cmd->pitch = controller->pitch.u;

	controller->yaw.setpoint = rate_desired->yaw;
	controller->yaw.measurement = gyro->z;
	rdd2_pid_axis_step(&controller->yaw, dt, integrate);
	rate_cmd->yaw = controller->yaw.u;
}

void rdd2_mix_quad_x(float throttle, const synapse_topic_RateTriplet_t *rate_cmd,
		     synapse_topic_MotorValues4f_t *motors)
{
	float max_up = 0.0f;
	float max_down = 0.0f;
	float correction[4];
	float scale = 1.0f;
	float *motor_values = rdd2_topic_motor_values_data(motors);

	for (size_t i = 0; i < 4U; i++) {
		correction[i] = (rate_cmd->roll * g_motor_mix[i].roll) +
				(rate_cmd->pitch * g_motor_mix[i].pitch) +
				(rate_cmd->yaw * g_motor_mix[i].yaw);

		if (correction[i] > max_up) {
			max_up = correction[i];
		}
		if (-correction[i] > max_down) {
			max_down = -correction[i];
		}
	}

	if (max_up > (1.0f - throttle) && max_up > 0.0f) {
		scale = (1.0f - throttle) / max_up;
	}

	if (max_down > throttle && max_down > 0.0f) {
		float down_scale = throttle / max_down;

		if (down_scale < scale) {
			scale = down_scale;
		}
	}

	for (size_t i = 0; i < 4U; i++) {
		motor_values[i] = clampf(throttle + (correction[i] * scale), 0.0f, 1.0f);
	}
}
