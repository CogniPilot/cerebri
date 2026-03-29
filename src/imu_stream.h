#ifndef CEREBRI_IMU_STREAM_H_
#define CEREBRI_IMU_STREAM_H_

#include "topic_flatbuffer.h"

#include <stdbool.h>

int cerebri_imu_stream_init(void);
bool cerebri_imu_stream_wait_next(cerebri_topic_Vec3f_t *gyro,
				  cerebri_topic_Vec3f_t *accel,
				  float *dt);

#endif
