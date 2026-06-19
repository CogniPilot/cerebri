/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_CUBS2_SITL)
#include "sitl_flatbuffer.h"
#include "sitl_udp_coordinator.h"
#endif

#include "generated_fixed_wing/CubControl_FixedWingOuterLoop.h"

LOG_MODULE_DECLARE(cubs2, LOG_LEVEL_INF);

K_SEM_DEFINE(g_input_sem, 0, 1);

static void timer_handler(struct k_timer *t)
{
	k_sem_give(&g_input_sem);
}

K_TIMER_DEFINE(g_ctrl_timer, timer_handler, NULL);

int cubs2_control_io_init(void)
{
	k_timer_start(&g_ctrl_timer, K_NO_WAIT,
		      K_NSEC(CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_NS));
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
	uint8_t rc_link_quality = 0U;
	bool rc_valid = false;
	bool imu_valid = false;

	// Wait for the next timer tick
	(void)k_sem_take(&g_input_sem, K_FOREVER);

	// Default values if no real sensor data is present
	*dt = (float)CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_S;
	status->imu_ok = true;
	status->rc_valid = true;

#if defined(CONFIG_CUBS2_SITL)
	uint8_t buf[CUBS2_SITL_INPUT_MAX_SIZE];
	size_t len;
	uint32_t generation;

	if (cubs2_sitl_udp_latest_input_get(buf, sizeof(buf), &len, &generation) &&
	    cubs2_sitl_fb_unpack_input(buf, len, gyro, accel, rc, &rc_link_quality, &rc_valid,
				       &imu_valid)) {
		status->rc_link_quality = rc_link_quality;
		status->rc_valid = rc_valid;
		status->imu_ok = imu_valid;
	}
#else
	ARG_UNUSED(rc_link_quality);
	ARG_UNUSED(rc_valid);
	ARG_UNUSED(imu_valid);
#endif
}
