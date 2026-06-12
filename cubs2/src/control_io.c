/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(cubs2, LOG_LEVEL_INF);

K_SEM_DEFINE(g_input_sem, 0, 1);

int cubs2_control_io_init(void)
{
	return 0;
}

void cubs2_control_input_trigger(void)
{
	k_sem_give(&g_input_sem);
}

void cubs2_control_input_wait(synapse_topic_Vec3f_t *gyro,
				synapse_topic_Vec3f_t *accel,
				synapse_topic_RcChannels16_t *rc,
				synapse_topic_ControlStatus_t *status,
				float *dt)
{
	// Wait for the next input trigger (e.g. from bridge or timer)
	(void)k_sem_take(&g_input_sem, K_MSEC(10));

	// Default values if no real sensor data is present
	*dt = 0.01f;
	status->imu_ok = true;
	status->rc_valid = true;
}
