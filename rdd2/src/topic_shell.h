#ifndef RDD2_TOPIC_SHELL_H_
#define RDD2_TOPIC_SHELL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

void rdd2_topic_rc_published(void);

void rdd2_topic_motor_output_publish(const synapse_topic_MotorValues4f_t *motors,
					 const synapse_topic_MotorRaw4u16_t *raw, bool armed,
					 bool test_mode);
bool rdd2_topic_motor_output_get(synapse_topic_MotorValues4f_t *motors,
				     synapse_topic_MotorRaw4u16_t *raw, bool *armed,
				     bool *test_mode);
bool rdd2_topic_motor_output_copy_blob(uint8_t *buf, size_t buf_size, size_t *len);
uint32_t rdd2_topic_motor_output_generation(void);

void rdd2_topic_flight_state_publish(
	const synapse_topic_Vec3f_t *gyro, const synapse_topic_Vec3f_t *accel,
	const synapse_topic_RcChannels16_t *rc, const synapse_topic_ControlStatus_t *status,
	const synapse_topic_AttitudeEuler_t *attitude,
	const synapse_topic_AttitudeEuler_t *attitude_desired,
	const synapse_topic_RateTriplet_t *rate_desired,
	const synapse_topic_RateTriplet_t *rate_cmd);
bool rdd2_topic_flight_state_get(
	synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel,
	synapse_topic_RcChannels16_t *rc, synapse_topic_ControlStatus_t *status,
	synapse_topic_AttitudeEuler_t *attitude,
	synapse_topic_AttitudeEuler_t *attitude_desired,
	synapse_topic_RateTriplet_t *rate_desired, synapse_topic_RateTriplet_t *rate_cmd);
bool rdd2_topic_flight_state_copy_blob(uint8_t *buf, size_t buf_size, size_t *len);
uint32_t rdd2_topic_flight_state_generation(void);

#endif
