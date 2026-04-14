#ifndef RDD2_MOTOR_OUTPUT_H_
#define RDD2_MOTOR_OUTPUT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

#define RDD2_MOTOR_IDLE_THROTTLE 0.0f

void rdd2_motor_output_init(void);
bool rdd2_motor_output_ready(void);

uint64_t rdd2_motor_output_write_all(const synapse_topic_MotorValues4f_t *motors, bool armed,
				     bool test_mode);
uint64_t rdd2_motor_output_write_all_raw(const synapse_topic_MotorRaw4u16_t *raw, bool test_mode);

bool rdd2_motor_test_get(synapse_topic_MotorValues4f_t *motors);
void rdd2_motor_test_set(size_t index, float value);
void rdd2_motor_test_clear(void);

bool rdd2_motor_raw_test_get(synapse_topic_MotorRaw4u16_t *raw);
void rdd2_motor_raw_test_set(size_t index, uint16_t value);
void rdd2_motor_raw_test_set_all(uint16_t value);
void rdd2_motor_raw_test_clear(void);

#endif
