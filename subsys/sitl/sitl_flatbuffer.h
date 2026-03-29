#ifndef CEREBRI_SITL_FLATBUFFER_H_
#define CEREBRI_SITL_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cerebri_sil_reader.h"

bool cerebri_sitl_fb_unpack_input(
	const uint8_t *buf, size_t buf_size, cerebri_topic_Vec3f_t *gyro,
	cerebri_topic_Vec3f_t *accel, cerebri_topic_RcChannels16_t *rc,
	uint8_t *rc_link_quality, bool *rc_valid, bool *imu_valid);

#endif
