#ifndef CEREBRI2_SITL_FLATBUFFER_H_
#define CEREBRI2_SITL_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_messages.h"

#define CEREBRI2_SITL_FB_SIM_INPUT_IDENTIFIER "C2SI"

struct sitl_input_msg {
	struct imu_sample imu;
	struct rc_channels_msg rc;
	uint8_t rc_link_quality;
	bool rc_valid;
	bool imu_valid;
};

bool cerebri2_sitl_fb_unpack_input(
	const uint8_t *buf, size_t buf_size, struct sitl_input_msg *input);

#endif
