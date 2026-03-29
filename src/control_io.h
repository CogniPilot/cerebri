#ifndef CEREBRI_CONTROL_IO_H_
#define CEREBRI_CONTROL_IO_H_

#include "topic_flatbuffer.h"

int cerebri_control_io_init(void);
void cerebri_control_input_wait(cerebri_topic_Vec3f_t *gyro,
				cerebri_topic_Vec3f_t *accel,
				cerebri_topic_RcChannels16_t *rc,
				cerebri_topic_ControlStatus_t *status,
				float *dt);

#endif
