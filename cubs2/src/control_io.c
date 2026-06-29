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

#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
#include "host_deploy_io.h"
#endif

LOG_MODULE_DECLARE(cubs2, LOG_LEVEL_INF);

K_SEM_DEFINE(g_input_sem, 0, 1);

int cubs2_control_io_init(void)
{
#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
	return cubs2_host_deploy_io_init();
#else
	return 0;
#endif
}

void cubs2_control_input_trigger(void)
{
	k_sem_give(&g_input_sem);
}

void cubs2_control_input_wait(synapse_topic_Vec3f_t *gyro,
				synapse_topic_Vec3f_t *accel,
				synapse_topic_RcChannels16_t *rc,
				synapse_topic_ControlStatus_t *status,
				float *dt,
				cubs2_mocap_rigid_body_t *mocap)
{
	uint8_t rc_link_quality = 0U;
	bool rc_valid = false;
	bool imu_valid = false;

#if !defined(CONFIG_CUBS2_SITL)
	// Flight: wait for the next input trigger (serial-bridge mocap) or 10 ms.
	(void)k_sem_take(&g_input_sem, K_MSEC(10));
#endif

	// Default values if no real sensor data is present
	*dt = 0.01f;
	status->imu_ok = true;
	status->rc_valid = true;

	// Default: no mocap from this source (flight build gets it from the
	// serial bridge instead).
	if (mocap != NULL) {
		*mocap = (cubs2_mocap_rigid_body_t){0};
	}

#if defined(CONFIG_CUBS2_SITL)
	uint8_t buf[CUBS2_SITL_INPUT_MAX_SIZE];
	size_t len = 0U;
	uint32_t generation = 0U;
	static uint32_t s_last_generation;   // last plant step we consumed
	bool have_input = false;

	// Lockstep: block until the plant publishes a NEW input (its generation
	// counter bumps), so the controller runs exactly once per simulator step
	// and the finite-difference dt (0.01 s) matches one real plant step. Two
	// guards keep it from deadlocking:
	//   * before any pose has arrived (generation == 0) proceed immediately, so
	//     the controller emits a first packet to prime the (also-waiting) plant;
	//   * a 1 s ceiling keeps a dead/paused peer from hanging the loop forever.
	static bool s_primed;   // emitted the initial control to prime the plant?
	const int64_t deadline = k_uptime_get() + 1000;
	while (true) {
		uint32_t gen;
		size_t l;

		if (cubs2_sitl_udp_latest_input_get(buf, sizeof(buf), &l, &gen)) {
			if (gen != s_last_generation) {
				s_last_generation = gen;
				len = l;
				have_input = true;
				s_primed = true;
				break;   // fresh plant step -> consume it
			}
		} else if (!s_primed) {
			s_primed = true;
			break;   // no pose has ever arrived -> emit one control to prime
		}
		if (k_uptime_get() >= deadline) {
			break;   // safety: peer stalled
		}
		// Yield so the SITL rx thread can drain the plant's reply. (This also
		// replaces the yield the old k_sem_take used to provide.)
		k_sleep(K_MSEC(1));
	}
	(void)generation;

	if (have_input &&
	    cubs2_sitl_fb_unpack_input(buf, len, gyro, accel, rc, &rc_link_quality, &rc_valid,
				       &imu_valid, mocap)) {
		status->rc_link_quality = rc_link_quality;
		status->rc_valid = rc_valid;
		status->imu_ok = imu_valid;
	}
#else
	ARG_UNUSED(rc_link_quality);
	ARG_UNUSED(rc_valid);
	ARG_UNUSED(imu_valid);
#endif

#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
	if (mocap != NULL) {
		(void)cubs2_host_deploy_io_get_mocap(mocap);
	}
	if (cubs2_host_deploy_io_get_rc(rc)) {
		status->rc_valid = true;
		status->rc_link_quality = 100U;
	} else {
		status->rc_valid = false;
		status->rc_link_quality = 0U;
	}
#endif
}
