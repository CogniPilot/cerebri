#ifndef RDD2_IMU_STREAM_H_
#define RDD2_IMU_STREAM_H_

#include "topic_flatbuffer.h"

#include <stdbool.h>

int rdd2_imu_stream_init(void);
bool rdd2_imu_stream_wait_next(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel, float *dt,
			       uint64_t *interrupt_timestamp_ns);

#endif
