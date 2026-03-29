#ifndef CEREBRI_TOPIC_SHELL_H_
#define CEREBRI_TOPIC_SHELL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

void cerebri_topic_rc_published(void);

void cerebri_topic_motor_output_publish(const cerebri_topic_MotorValues4f_t *motors,
					 const cerebri_topic_MotorRaw4u16_t *raw, bool armed,
					 bool test_mode);
bool cerebri_topic_motor_output_get(cerebri_topic_MotorValues4f_t *motors,
				     cerebri_topic_MotorRaw4u16_t *raw, bool *armed,
				     bool *test_mode);
bool cerebri_topic_motor_output_copy_blob(uint8_t *buf, size_t buf_size, size_t *len);
uint32_t cerebri_topic_motor_output_generation(void);

void cerebri_topic_flight_state_publish(
	const cerebri_topic_Vec3f_t *gyro, const cerebri_topic_Vec3f_t *accel,
	const cerebri_topic_RcChannels16_t *rc, const cerebri_topic_ControlStatus_t *status,
	const cerebri_topic_RateTriplet_t *rate_desired,
	const cerebri_topic_RateTriplet_t *rate_cmd);
bool cerebri_topic_flight_state_get(
	cerebri_topic_Vec3f_t *gyro, cerebri_topic_Vec3f_t *accel,
	cerebri_topic_RcChannels16_t *rc, cerebri_topic_ControlStatus_t *status,
	cerebri_topic_RateTriplet_t *rate_desired, cerebri_topic_RateTriplet_t *rate_cmd);
bool cerebri_topic_flight_state_copy_blob(uint8_t *buf, size_t buf_size, size_t *len);
uint32_t cerebri_topic_flight_state_generation(void);

#endif
