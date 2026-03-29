#ifndef CEREBRI_MOTOR_OUTPUT_H_
#define CEREBRI_MOTOR_OUTPUT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

#define CEREBRI_MOTOR_IDLE_THROTTLE 0.0f

void cerebri_motor_output_init(void);
bool cerebri_motor_output_ready(void);

void cerebri_motor_output_write_all(const cerebri_topic_MotorValues4f_t *motors, bool armed,
				     bool test_mode);
void cerebri_motor_output_write_all_raw(const cerebri_topic_MotorRaw4u16_t *raw, bool test_mode);

bool cerebri_motor_test_get(cerebri_topic_MotorValues4f_t *motors);
void cerebri_motor_test_set(size_t index, float value);
void cerebri_motor_test_clear(void);

bool cerebri_motor_raw_test_get(cerebri_topic_MotorRaw4u16_t *raw);
void cerebri_motor_raw_test_set(size_t index, uint16_t value);
void cerebri_motor_raw_test_set_all(uint16_t value);
void cerebri_motor_raw_test_clear(void);

#endif
