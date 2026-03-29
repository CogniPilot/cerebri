/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rate_control.h"

#include "motor_output.h"

#define RC_US_CENTER              1500
#define RC_US_MIN                 1000
#define RC_US_MAX                 2000
#define ARM_SWITCH_THRESHOLD_US   1600
#define ARM_CHANNEL_INDEX         4
#define ROLL_CHANNEL_INDEX        0
#define PITCH_CHANNEL_INDEX       1
#define THROTTLE_CHANNEL_INDEX    2
#define YAW_CHANNEL_INDEX         3
#define MAX_ROLL_PITCH_RATE_RAD_S 6.0f
#define MAX_YAW_RATE_RAD_S        3.5f
#define PID_OUTPUT_LIMIT          0.35f
#define PID_I_LIMIT               0.20f
#define DTERM_LPF_ALPHA           0.15f

struct motor_mix {
	float roll;
	float pitch;
	float yaw;
};

/*
 * Default mixer uses FRD body axes and assumes DSHOT channels 1..4 are:
 * front-right, rear-right, rear-left, front-left.
 *
 * Positive roll is about body +x (forward), so left motors increase and
 * right motors decrease.
 *
 * Positive pitch is about body +y (right), so front motors increase and rear
 * motors decrease.
 *
 * Yaw signs depend on motor spin direction and should be bench-verified on
 * hardware.
 */
static const struct motor_mix g_motor_mix[4] = {
	{ .roll = -1.0f, .pitch = 1.0f, .yaw = 1.0f },
	{ .roll = -1.0f, .pitch = -1.0f, .yaw = -1.0f },
	{ .roll = 1.0f, .pitch = -1.0f, .yaw = 1.0f },
	{ .roll = 1.0f, .pitch = 1.0f, .yaw = -1.0f },
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

static void pid_reset(struct cerebri_pid_axis *pid)
{
	pid->integrator = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->derivative_lpf = 0.0f;
}

static float pid_step(struct cerebri_pid_axis *pid, float setpoint, float measurement, float dt,
		      bool integrate)
{
	float error = setpoint - measurement;
	float derivative = (measurement - pid->prev_measurement) / dt;

	pid->prev_measurement = measurement;
	pid->derivative_lpf += DTERM_LPF_ALPHA * (derivative - pid->derivative_lpf);

	if (integrate) {
		pid->integrator += error * pid->ki * dt;
		pid->integrator = clampf(pid->integrator, -PID_I_LIMIT, PID_I_LIMIT);
	} else {
		pid->integrator = 0.0f;
	}

	return clampf((pid->kp * error) + pid->integrator - (pid->kd * pid->derivative_lpf),
		      -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
}

void cerebri_rate_controller_init(struct cerebri_rate_controller *controller)
{
	*controller = (struct cerebri_rate_controller){
		.roll = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f },
		.pitch = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f },
		.yaw = { .kp = 0.20f, .ki = 0.20f, .kd = 0.0000f },
	};
	cerebri_rate_controller_reset(controller);
}

void cerebri_rate_controller_reset(struct cerebri_rate_controller *controller)
{
	pid_reset(&controller->roll);
	pid_reset(&controller->pitch);
	pid_reset(&controller->yaw);
}

bool cerebri_rate_arm_switch_high(const cerebri_topic_RcChannels16_t *rc)
{
	return cerebri_topic_rc_channels_data_const(rc)[ARM_CHANNEL_INDEX] > ARM_SWITCH_THRESHOLD_US;
}

int32_t cerebri_rate_throttle_us(const cerebri_topic_RcChannels16_t *rc)
{
	return cerebri_topic_rc_channels_data_const(rc)[THROTTLE_CHANNEL_INDEX];
}

float cerebri_rate_throttle_input_from_rc(const cerebri_topic_RcChannels16_t *rc)
{
	return rc_norm_throttle(cerebri_rate_throttle_us(rc));
}

float cerebri_rate_throttle_command(float throttle_input, bool armed)
{
	if (!armed) {
		return 0.0f;
	}

	return CEREBRI_MOTOR_IDLE_THROTTLE +
	       (throttle_input * (1.0f - CEREBRI_MOTOR_IDLE_THROTTLE));
}

void cerebri_rate_desired_from_rc(const cerebri_topic_RcChannels16_t *rc,
				  cerebri_topic_RateTriplet_t *rate_desired)
{
	const int32_t *channels = cerebri_topic_rc_channels_data_const(rc);

	rate_desired->roll =
		rc_norm_centered(channels[ROLL_CHANNEL_INDEX]) * MAX_ROLL_PITCH_RATE_RAD_S;
	rate_desired->pitch =
		-1.0f * rc_norm_centered(channels[PITCH_CHANNEL_INDEX]) *
		MAX_ROLL_PITCH_RATE_RAD_S;
	rate_desired->yaw =
		rc_norm_centered(channels[YAW_CHANNEL_INDEX]) * MAX_YAW_RATE_RAD_S;
}

void cerebri_rate_controller_step(struct cerebri_rate_controller *controller,
				  const cerebri_topic_RateTriplet_t *rate_desired,
				  const cerebri_topic_Vec3f_t *gyro, float dt,
				  bool integrate,
				  cerebri_topic_RateTriplet_t *rate_cmd)
{
	if (dt <= 0.0f) {
		*rate_cmd = (cerebri_topic_RateTriplet_t){0};
		return;
	}

	rate_cmd->roll = pid_step(&controller->roll, rate_desired->roll, gyro->x, dt, integrate);
	rate_cmd->pitch =
		pid_step(&controller->pitch, rate_desired->pitch, gyro->y, dt, integrate);
	rate_cmd->yaw = pid_step(&controller->yaw, rate_desired->yaw, gyro->z, dt, integrate);
}

void cerebri_mix_quad_x(float throttle, const cerebri_topic_RateTriplet_t *rate_cmd,
			cerebri_topic_MotorValues4f_t *motors)
{
	float max_up = 0.0f;
	float max_down = 0.0f;
	float correction[4];
	float scale = 1.0f;
	float *motor_values = cerebri_topic_motor_values_data(motors);

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
