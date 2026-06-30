#ifndef CUBS2_CONTROL_IO_H_
#define CUBS2_CONTROL_IO_H_

#include "topic_flatbuffer.h"

#include <zephyr/kernel.h>

int cubs2_control_io_init(void);
void cubs2_control_input_wait(synapse_topic_Vec3f_t *gyro,
				synapse_topic_Vec3f_t *accel,
				synapse_topic_RcChannels16_t *rc,
				synapse_topic_ControlStatus_t *status,
				float controller_period_s,
				k_timeout_t controller_timeout,
				float *dt,
				cubs2_mocap_rigid_body_t *mocap);

void cubs2_control_input_trigger(void);

#endif
