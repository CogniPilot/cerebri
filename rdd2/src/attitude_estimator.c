/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_estimator.h"

#include <math.h>
#include <string.h>

#define RDD2_GRAVITY_M_S2           9.81f
#define RDD2_ATTITUDE_CORRECTION_KP 0.35f
#define RDD2_ATTITUDE_CORRECTION_KI 0.02f
#define RDD2_ATTITUDE_ACCEL_MIN_G   0.6f
#define RDD2_ATTITUDE_ACCEL_MAX_G   1.4f

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

static void quaternion_normalize(struct rdd2_attitude_estimator *estimator)
{
	float norm_sq = (estimator->q_w * estimator->q_w) + (estimator->q_x * estimator->q_x) +
			(estimator->q_y * estimator->q_y) + (estimator->q_z * estimator->q_z);
	float inv_norm;

	if (norm_sq <= 1.0e-6f) {
		estimator->q_w = 1.0f;
		estimator->q_x = 0.0f;
		estimator->q_y = 0.0f;
		estimator->q_z = 0.0f;
		return;
	}

	inv_norm = 1.0f / sqrtf(norm_sq);
	estimator->q_w *= inv_norm;
	estimator->q_x *= inv_norm;
	estimator->q_y *= inv_norm;
	estimator->q_z *= inv_norm;
}

static void quaternion_from_euler(struct rdd2_attitude_estimator *estimator, float roll,
				  float pitch, float yaw)
{
	float cr = cosf(roll * 0.5f);
	float sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f);
	float sp = sinf(pitch * 0.5f);
	float cy = cosf(yaw * 0.5f);
	float sy = sinf(yaw * 0.5f);

	estimator->q_w = (cy * cp * cr) + (sy * sp * sr);
	estimator->q_x = (cy * cp * sr) - (sy * sp * cr);
	estimator->q_y = (sy * cp * sr) + (cy * sp * cr);
	estimator->q_z = (sy * cp * cr) - (cy * sp * sr);
	quaternion_normalize(estimator);
}

static void attitude_from_quaternion(const struct rdd2_attitude_estimator *estimator,
				     synapse_topic_AttitudeEuler_t *attitude)
{
	float sinr_cosp =
		(2.0f * estimator->q_w * estimator->q_x) + (2.0f * estimator->q_y * estimator->q_z);
	float cosr_cosp =
		1.0f -
		(2.0f * ((estimator->q_x * estimator->q_x) + (estimator->q_y * estimator->q_y)));
	float sinp =
		(2.0f * estimator->q_w * estimator->q_y) - (2.0f * estimator->q_z * estimator->q_x);
	float siny_cosp =
		(2.0f * estimator->q_w * estimator->q_z) + (2.0f * estimator->q_x * estimator->q_y);
	float cosy_cosp =
		1.0f -
		(2.0f * ((estimator->q_y * estimator->q_y) + (estimator->q_z * estimator->q_z)));

	attitude->roll = atan2f(sinr_cosp, cosr_cosp);
	attitude->pitch = asinf(clampf(sinp, -1.0f, 1.0f));
	attitude->yaw = atan2f(siny_cosp, cosy_cosp);
}

static bool accel_correction_valid(const synapse_topic_Vec3f_t *accel, float *ax, float *ay,
				   float *az)
{
	float accel_norm;

	accel_norm = sqrtf((accel->x * accel->x) + (accel->y * accel->y) + (accel->z * accel->z));
	if (accel_norm < (RDD2_ATTITUDE_ACCEL_MIN_G * RDD2_GRAVITY_M_S2) ||
	    accel_norm > (RDD2_ATTITUDE_ACCEL_MAX_G * RDD2_GRAVITY_M_S2)) {
		return false;
	}

	*ax = accel->x / accel_norm;
	*ay = accel->y / accel_norm;
	*az = accel->z / accel_norm;
	return true;
}

void rdd2_attitude_estimator_init(struct rdd2_attitude_estimator *estimator)
{
	if (estimator == NULL) {
		return;
	}

	memset(estimator, 0, sizeof(*estimator));
	estimator->q_w = 1.0f;
}

void rdd2_attitude_estimator_reset_from_accel(struct rdd2_attitude_estimator *estimator,
					      const synapse_topic_Vec3f_t *accel)
{
	float roll = 0.0f;
	float pitch = 0.0f;

	if (estimator == NULL) {
		return;
	}

	memset(estimator, 0, sizeof(*estimator));
	if (accel != NULL) {
		float yz_norm = sqrtf((accel->y * accel->y) + (accel->z * accel->z));

		roll = atan2f(accel->y, accel->z);
		pitch = atan2f(-accel->x, yz_norm);
	}

	quaternion_from_euler(estimator, roll, pitch, 0.0f);
}

void rdd2_attitude_estimator_predict(struct rdd2_attitude_estimator *estimator,
				     const synapse_topic_Vec3f_t *gyro,
				     const synapse_topic_Vec3f_t *accel, float dt)
{
	float gx;
	float gy;
	float gz;
	float q_w;
	float q_x;
	float q_y;
	float q_z;

	if (estimator == NULL || gyro == NULL || accel == NULL || dt <= 0.0f) {
		return;
	}

	gx = gyro->x;
	gy = gyro->y;
	gz = gyro->z;

	if (accel != NULL) {
		float ax;
		float ay;
		float az;

		if (accel_correction_valid(accel, &ax, &ay, &az)) {
			float gravity_x = 2.0f * ((estimator->q_x * estimator->q_z) -
						  (estimator->q_w * estimator->q_y));
			float gravity_y = 2.0f * ((estimator->q_w * estimator->q_x) +
						  (estimator->q_y * estimator->q_z));
			float gravity_z = (estimator->q_w * estimator->q_w) -
					  (estimator->q_x * estimator->q_x) -
					  (estimator->q_y * estimator->q_y) +
					  (estimator->q_z * estimator->q_z);
			float error_x = (ay * gravity_z) - (az * gravity_y);
			float error_y = (az * gravity_x) - (ax * gravity_z);
			float error_z = (ax * gravity_y) - (ay * gravity_x);

			estimator->gyro_bias_x += RDD2_ATTITUDE_CORRECTION_KI * error_x * dt;
			estimator->gyro_bias_y += RDD2_ATTITUDE_CORRECTION_KI * error_y * dt;
			estimator->gyro_bias_z += RDD2_ATTITUDE_CORRECTION_KI * error_z * dt;

			gx += estimator->gyro_bias_x + (RDD2_ATTITUDE_CORRECTION_KP * error_x);
			gy += estimator->gyro_bias_y + (RDD2_ATTITUDE_CORRECTION_KP * error_y);
			gz += estimator->gyro_bias_z + (RDD2_ATTITUDE_CORRECTION_KP * error_z);
		} else {
			gx += estimator->gyro_bias_x;
			gy += estimator->gyro_bias_y;
			gz += estimator->gyro_bias_z;
		}
	}

	q_w = estimator->q_w;
	q_x = estimator->q_x;
	q_y = estimator->q_y;
	q_z = estimator->q_z;

	estimator->q_w += 0.5f * (-q_x * gx - q_y * gy - q_z * gz) * dt;
	estimator->q_x += 0.5f * (q_w * gx + q_y * gz - q_z * gy) * dt;
	estimator->q_y += 0.5f * (q_w * gy - q_x * gz + q_z * gx) * dt;
	estimator->q_z += 0.5f * (q_w * gz + q_x * gy - q_y * gx) * dt;

	quaternion_normalize(estimator);
}

void rdd2_attitude_estimator_get_attitude(const struct rdd2_attitude_estimator *estimator,
					  synapse_topic_AttitudeEuler_t *attitude)
{
	if (estimator == NULL || attitude == NULL) {
		return;
	}

	attitude_from_quaternion(estimator, attitude);
}
