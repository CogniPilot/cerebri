#ifndef CEREBRI_TOPIC_FLATBUFFER_H_
#define CEREBRI_TOPIC_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cerebri_topics_reader.h"

#define CEREBRI_TOPIC_FB_FLIGHT_STATE_SIZE 164U
#define CEREBRI_TOPIC_FB_MOTOR_OUTPUT_SIZE  48U

static inline float *cerebri_topic_vec3f_data(cerebri_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline const float *cerebri_topic_vec3f_data_const(const cerebri_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline int32_t *cerebri_topic_rc_channels_data(cerebri_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline const int32_t *cerebri_topic_rc_channels_data_const(
	const cerebri_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline float *cerebri_topic_rate_triplet_data(cerebri_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline const float *cerebri_topic_rate_triplet_data_const(
	const cerebri_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline float *cerebri_topic_motor_values_data(cerebri_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline const float *cerebri_topic_motor_values_data_const(
	const cerebri_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline uint16_t *cerebri_topic_motor_raw_data(cerebri_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

static inline const uint16_t *cerebri_topic_motor_raw_data_const(
	const cerebri_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

size_t cerebri_topic_fb_pack_flight_state(
	uint8_t *buf, size_t buf_size, const cerebri_topic_Vec3f_t *gyro,
	const cerebri_topic_Vec3f_t *accel, const cerebri_topic_RcChannels16_t *rc,
	const cerebri_topic_ControlStatus_t *status,
	const cerebri_topic_RateTriplet_t *rate_desired,
	const cerebri_topic_RateTriplet_t *rate_cmd);

bool cerebri_topic_fb_unpack_flight_state(
	const uint8_t *buf, size_t buf_size, cerebri_topic_Vec3f_t *gyro,
	cerebri_topic_Vec3f_t *accel, cerebri_topic_RcChannels16_t *rc,
	cerebri_topic_ControlStatus_t *status, cerebri_topic_RateTriplet_t *rate_desired,
	cerebri_topic_RateTriplet_t *rate_cmd);

size_t cerebri_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const cerebri_topic_MotorValues4f_t *motors,
	const cerebri_topic_MotorRaw4u16_t *raw, bool armed, bool test_mode);

bool cerebri_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, cerebri_topic_MotorValues4f_t *motors,
	cerebri_topic_MotorRaw4u16_t *raw, bool *armed, bool *test_mode);

#endif
