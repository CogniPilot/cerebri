#ifndef CUBS2_TOPIC_FLATBUFFER_H_
#define CUBS2_TOPIC_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "synapse_topics_reader.h"
#include "synapse_mocap_reader.h"

#define CUBS2_TOPIC_FB_FLIGHT_STATE_SIZE 192U
#define CUBS2_TOPIC_FB_MOTOR_OUTPUT_SIZE  48U
#define CUBS2_TOPIC_FB_MOCAP_FRAME_MAX_SIZE 512U

typedef struct {
    float x;
    float y;
    float z;
    float qw;
    float qx;
    float qy;
    float qz;
    bool valid;
} cubs2_mocap_rigid_body_t;

static inline float *cubs2_topic_vec3f_data(synapse_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline const float *cubs2_topic_vec3f_data_const(const synapse_topic_Vec3f_t *vec)
{
	return &vec->x;
}

static inline int32_t *cubs2_topic_rc_channels_data(synapse_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline const int32_t *cubs2_topic_rc_channels_data_const(
	const synapse_topic_RcChannels16_t *rc)
{
	return &rc->ch0;
}

static inline float *cubs2_topic_rate_triplet_data(synapse_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline const float *cubs2_topic_rate_triplet_data_const(
	const synapse_topic_RateTriplet_t *rate)
{
	return &rate->roll;
}

static inline float *cubs2_topic_attitude_euler_data(synapse_topic_AttitudeEuler_t *attitude)
{
	return &attitude->roll;
}

static inline const float *cubs2_topic_attitude_euler_data_const(
	const synapse_topic_AttitudeEuler_t *attitude)
{
	return &attitude->roll;
}

static inline float *cubs2_topic_motor_values_data(synapse_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline const float *cubs2_topic_motor_values_data_const(
	const synapse_topic_MotorValues4f_t *motors)
{
	return &motors->m0;
}

static inline uint16_t *cubs2_topic_motor_raw_data(synapse_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

static inline const uint16_t *cubs2_topic_motor_raw_data_const(
	const synapse_topic_MotorRaw4u16_t *raw)
{
	return &raw->m0;
}

size_t cubs2_topic_fb_pack_flight_state(
	uint8_t *buf, size_t buf_size, const synapse_topic_Vec3f_t *gyro,
	const synapse_topic_Vec3f_t *accel, const synapse_topic_RcChannels16_t *rc,
	const synapse_topic_ControlStatus_t *status,
	const synapse_topic_AttitudeEuler_t *attitude,
	const synapse_topic_AttitudeEuler_t *attitude_desired,
	const synapse_topic_RateTriplet_t *rate_desired,
	const synapse_topic_RateTriplet_t *rate_cmd);

bool cubs2_topic_fb_unpack_flight_state(
	const uint8_t *buf, size_t buf_size, synapse_topic_Vec3f_t *gyro,
	synapse_topic_Vec3f_t *accel, synapse_topic_RcChannels16_t *rc,
	synapse_topic_ControlStatus_t *status, synapse_topic_AttitudeEuler_t *attitude,
	synapse_topic_AttitudeEuler_t *attitude_desired,
	synapse_topic_RateTriplet_t *rate_desired, synapse_topic_RateTriplet_t *rate_cmd);

size_t cubs2_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const synapse_topic_MotorValues4f_t *motors,
	const synapse_topic_MotorRaw4u16_t *raw, bool armed, bool test_mode);

bool cubs2_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, synapse_topic_MotorValues4f_t *motors,
	synapse_topic_MotorRaw4u16_t *raw, bool *armed, bool *test_mode);

bool cubs2_topic_fb_unpack_mocap_frame(
	const uint8_t *buf, size_t buf_size,
	cubs2_mocap_rigid_body_t *rb);

bool cubs2_topic_fb_unpack_manual_control(
	const uint8_t *buf, size_t buf_size, synapse_topic_RcChannels16_t *rc,
	bool *valid);

#endif
