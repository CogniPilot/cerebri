#ifndef CEREBRI2_TOPIC_SHELL_H_
#define CEREBRI2_TOPIC_SHELL_H_

#include "topic_messages.h"

#include <stdint.h>

void cerebri2_topic_rc_published(void);
void cerebri2_topic_motor_output_update(const float motors[4], const uint16_t raw[4], bool armed,
					bool test_mode);
void cerebri2_topic_motor_output_get(float motors[4], uint16_t raw[4], bool *armed,
				     bool *test_mode);
struct motor_output_msg cerebri2_topic_motor_output_snapshot(void);
uint32_t cerebri2_topic_motor_output_generation(void);
void cerebri2_topic_flight_snapshot_update(const struct flight_snapshot *snapshot);
struct flight_snapshot cerebri2_topic_flight_snapshot_get(void);
uint32_t cerebri2_topic_flight_snapshot_generation(void);

#endif
