#ifndef RDD2_SITL_FLATBUFFER_H_
#define RDD2_SITL_FLATBUFFER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "synapse_sil_reader.h"

bool rdd2_sitl_fb_unpack_input(const uint8_t *buf, size_t buf_size, synapse_topic_Vec3f_t *gyro,
			       synapse_topic_Vec3f_t *accel, synapse_topic_RcChannels16_t *rc,
			       uint8_t *rc_link_quality, bool *rc_valid, bool *imu_valid);

#endif
