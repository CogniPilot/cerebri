#ifndef CEREBRI2_TOPIC_FLATBUFFER_H_
#define CEREBRI2_TOPIC_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_messages.h"

#define CEREBRI2_TOPIC_FB_FLIGHT_SNAPSHOT_SIZE 164U
#define CEREBRI2_TOPIC_FB_MOTOR_OUTPUT_SIZE     48U

size_t cerebri2_topic_fb_pack_flight_snapshot(
	uint8_t *buf, size_t buf_size, const struct flight_snapshot *snapshot);
bool cerebri2_topic_fb_unpack_flight_snapshot(
	const uint8_t *buf, size_t buf_size, struct flight_snapshot *snapshot);

size_t cerebri2_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const struct motor_output_msg *output);
bool cerebri2_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, struct motor_output_msg *output);

#endif
