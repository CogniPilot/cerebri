#ifndef RDD2_TOPIC_FLATBUFFER_H_
#define RDD2_TOPIC_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "synapse_topics_reader.h"

#define RDD2_TOPIC_FB_FLIGHT_STATE_SIZE 192U
#define RDD2_TOPIC_FB_MOTOR_OUTPUT_SIZE  48U

static inline float *rdd2_topic_vec3f_data(synapse_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline const float *rdd2_topic_vec3f_data_const(const synapse_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline int32_t *rdd2_topic_rc_channels_data(synapse_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline const int32_t *rdd2_topic_rc_channels_data_const(
	const synapse_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline float *rdd2_topic_rate_triplet_data(synapse_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline const float *rdd2_topic_rate_triplet_data_const(
	const synapse_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline float *rdd2_topic_attitude_euler_data(synapse_topic_AttitudeEuler_t *attitude)
{
	return &attitude->roll;
}

static inline const float *rdd2_topic_attitude_euler_data_const(
	const synapse_topic_AttitudeEuler_t *attitude)
{
	return &attitude->roll;
}

static inline float *rdd2_topic_motor_values_data(synapse_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline const float *rdd2_topic_motor_values_data_const(
	const synapse_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline uint16_t *rdd2_topic_motor_raw_data(synapse_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

static inline const uint16_t *rdd2_topic_motor_raw_data_const(
	const synapse_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

size_t rdd2_topic_fb_pack_flight_state(
	uint8_t *buf, size_t buf_size, const synapse_topic_Vec3f_t *gyro,
	const synapse_topic_Vec3f_t *accel, const synapse_topic_RcChannels16_t *rc,
	const synapse_topic_ControlStatus_t *status,
	const synapse_topic_AttitudeEuler_t *attitude,
	const synapse_topic_AttitudeEuler_t *attitude_desired,
	const synapse_topic_RateTriplet_t *rate_desired,
	const synapse_topic_RateTriplet_t *rate_cmd);

bool rdd2_topic_fb_unpack_flight_state(
	const uint8_t *buf, size_t buf_size, synapse_topic_Vec3f_t *gyro,
	synapse_topic_Vec3f_t *accel, synapse_topic_RcChannels16_t *rc,
	synapse_topic_ControlStatus_t *status, synapse_topic_AttitudeEuler_t *attitude,
	synapse_topic_AttitudeEuler_t *attitude_desired,
	synapse_topic_RateTriplet_t *rate_desired, synapse_topic_RateTriplet_t *rate_cmd);

size_t rdd2_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const synapse_topic_MotorValues4f_t *motors,
	const synapse_topic_MotorRaw4u16_t *raw, bool armed, bool test_mode);

bool rdd2_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, synapse_topic_MotorValues4f_t *motors,
	synapse_topic_MotorRaw4u16_t *raw, bool *armed, bool *test_mode);

#endif
