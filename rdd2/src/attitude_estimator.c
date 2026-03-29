/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "attitude_estimator.h"

#include "generated/casadi.h"
#include "generated/rdd2.h"

#include <math.h>
#include <string.h>

#define RDD2_GRAVITY_M_S2 9.81f

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

static void quaternion_normalize(float q[4])
{
	float norm_sq = (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);
	float inv_norm;

	if (norm_sq <= 1.0e-6f) {
		q[0] = 1.0f;
		q[1] = 0.0f;
		q[2] = 0.0f;
		q[3] = 0.0f;
		return;
	}

	inv_norm = 1.0f / sqrtf(norm_sq);
	q[0] *= inv_norm;
	q[1] *= inv_norm;
	q[2] *= inv_norm;
	q[3] *= inv_norm;
}

static void quaternion_from_euler(float roll, float pitch, float yaw, float q[4])
{
	float cr = cosf(roll * 0.5f);
	float sr = sinf(roll * 0.5f);
	float cp = cosf(pitch * 0.5f);
	float sp = sinf(pitch * 0.5f);
	float cy = cosf(yaw * 0.5f);
	float sy = sinf(yaw * 0.5f);

	q[0] = (cy * cp * cr) + (sy * sp * sr);
	q[1] = (cy * cp * sr) - (sy * sp * cr);
	q[2] = (sy * cp * sr) + (cy * sp * cr);
	q[3] = (sy * cp * cr) - (cy * sp * sr);
	quaternion_normalize(q);
}

static void attitude_from_quaternion(const float q[4], synapse_topic_AttitudeEuler_t *attitude)
{
	float sinr_cosp = (2.0f * q[0] * q[1]) + (2.0f * q[2] * q[3]);
	float cosr_cosp = 1.0f - (2.0f * ((q[1] * q[1]) + (q[2] * q[2])));
	float sinp = (2.0f * q[0] * q[2]) - (2.0f * q[3] * q[1]);
	float siny_cosp = (2.0f * q[0] * q[3]) + (2.0f * q[1] * q[2]);
	float cosy_cosp = 1.0f - (2.0f * ((q[2] * q[2]) + (q[3] * q[3])));

	attitude->roll = atan2f(sinr_cosp, cosr_cosp);
	attitude->pitch = asinf(clampf(sinp, -1.0f, 1.0f));
	attitude->yaw = atan2f(siny_cosp, cosy_cosp);
}

void rdd2_attitude_estimator_init(struct rdd2_attitude_estimator *estimator)
{
	if (estimator == NULL) {
		return;
	}

	memset(estimator, 0, sizeof(*estimator));
	estimator->x[6] = 1.0f;
}

void rdd2_attitude_estimator_reset_from_accel(
	struct rdd2_attitude_estimator *estimator, const synapse_topic_Vec3f_t *accel)
{
	float roll = 0.0f;
	float pitch = 0.0f;

	if (estimator == NULL) {
		return;
	}

	memset(estimator->x, 0, sizeof(estimator->x));
	if (accel != NULL) {
		float yz_norm = sqrtf((accel->y * accel->y) + (accel->z * accel->z));

		roll = atan2f(accel->y, accel->z);
		pitch = atan2f(-accel->x, yz_norm);
	}

	quaternion_from_euler(roll, pitch, 0.0f, &estimator->x[6]);
}

void rdd2_attitude_estimator_predict(struct rdd2_attitude_estimator *estimator,
					const synapse_topic_Vec3f_t *gyro,
					const synapse_topic_Vec3f_t *accel, float dt)
{
	float a_b[3];
	float omega_b[3];
	float x1[10];
	float gravity = RDD2_GRAVITY_M_S2;

	if (estimator == NULL || gyro == NULL || accel == NULL || dt <= 0.0f) {
		return;
	}

	a_b[0] = accel->x;
	a_b[1] = accel->y;
	a_b[2] = accel->z;
	omega_b[0] = gyro->x;
	omega_b[1] = gyro->y;
	omega_b[2] = gyro->z;

	{
		CASADI_FUNC_ARGS(strapdown_ins_propagate);

		args[0] = estimator->x;
		args[1] = a_b;
		args[2] = omega_b;
		args[3] = &gravity;
		args[4] = &dt;
		res[0] = x1;
		CASADI_FUNC_CALL(strapdown_ins_propagate);
	}

	memcpy(estimator->x, x1, sizeof(x1));
	quaternion_normalize(&estimator->x[6]);
}

void rdd2_attitude_estimator_get_attitude(
	const struct rdd2_attitude_estimator *estimator,
	synapse_topic_AttitudeEuler_t *attitude)
{
	if (estimator == NULL || attitude == NULL) {
		return;
	}

	attitude_from_quaternion(&estimator->x[6], attitude);
}
