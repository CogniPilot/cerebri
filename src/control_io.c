/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "control_io.h"

#include "imu_stream.h"
#include "rc_input.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(cerebri, LOG_LEVEL_INF);

#define RC_NODE    DT_ALIAS(rc)
#define IMU_NODE   DT_ALIAS(imu0)
#define DSHOT_NODE DT_ALIAS(motors)
#define GNSS_NODE  DT_ALIAS(gnss)

static bool ready_or_log(const struct device *dev, const char *name)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("%s not ready", name);
		return false;
	}

	return true;
}

int cerebri_control_io_init(void)
{
	const struct device *const rc_dev = DEVICE_DT_GET(RC_NODE);
	const struct device *const imu_dev = DEVICE_DT_GET(IMU_NODE);
	const struct device *const dshot_dev = DEVICE_DT_GET(DSHOT_NODE);
	const struct device *const gnss_dev = DEVICE_DT_GET_OR_NULL(GNSS_NODE);

	cerebri_rc_input_init();

	if (!ready_or_log(dshot_dev, "dshot")) {
		return -ENODEV;
	}

	ready_or_log(rc_dev, "rc");
	ready_or_log(imu_dev, "imu");

	if (gnss_dev != NULL && device_is_ready(gnss_dev)) {
		LOG_INF("gnss path ready");
	}

	if (nxp_flexio_dshot_channel_count(dshot_dev) != 4U) {
		LOG_ERR("expected 4 dshot channels");
		return -EINVAL;
	}

	return cerebri_imu_stream_init();
}

void cerebri_control_input_wait(cerebri_topic_Vec3f_t *gyro,
				cerebri_topic_Vec3f_t *accel,
				cerebri_topic_RcChannels16_t *rc,
				cerebri_topic_ControlStatus_t *status,
				float *dt)
{
	bool rc_valid;

	status->imu_ok = cerebri_imu_stream_wait_next(gyro, accel, dt);
	status->rc_link_quality = cerebri_rc_input_link_quality_get(DEVICE_DT_GET(RC_NODE));
	cerebri_rc_input_latest_get(rc, &status->rc_stamp_ms, &rc_valid);
	status->rc_valid = rc_valid;
}
