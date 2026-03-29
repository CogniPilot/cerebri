/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cognipilot_sitl_imu

#include "sitl_flatbuffer.h"
#include "sitl_udp_coordinator.h"

#include <errno.h>

#include <zephyr/drivers/sensor.h>

struct sitl_imu_data {
	struct imu_sample sample;
	uint32_t generation;
	bool valid;
};

static int sitl_imu_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static int sitl_imu_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct sitl_imu_data *data = dev->data;
	uint8_t buf[CEREBRI2_SITL_INPUT_MAX_SIZE];
	size_t len;
	uint32_t generation;
	struct sitl_input_msg input;

	ARG_UNUSED(chan);

	if (!cerebri2_sitl_udp_latest_input_get(buf, sizeof(buf), &len, &generation)) {
		data->valid = false;
		return -ENODATA;
	}

	if (generation == data->generation) {
		return data->valid ? 0 : -ENODATA;
	}

	if (!cerebri2_sitl_fb_unpack_input(buf, len, &input)) {
		data->valid = false;
		data->generation = generation;
		return -ENODATA;
	}

	data->sample = input.imu;
	data->generation = generation;
	data->valid = input.imu_valid;

	return data->valid ? 0 : -ENODATA;
}

static int sitl_imu_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	const struct sitl_imu_data *data = dev->data;
	float sensor_axes[3];

	if (!data->valid) {
		return -ENODATA;
	}

	switch (chan) {
	case SENSOR_CHAN_GYRO_XYZ:
		sensor_axes[0] = data->sample.gyro_rad_s[1];
		sensor_axes[1] = data->sample.gyro_rad_s[0];
		sensor_axes[2] = -data->sample.gyro_rad_s[2];
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		sensor_axes[0] = data->sample.accel_m_s2[1];
		sensor_axes[1] = data->sample.accel_m_s2[0];
		sensor_axes[2] = -data->sample.accel_m_s2[2];
		break;
	default:
		return -EINVAL;
	}

	for (size_t i = 0; i < 3U; i++) {
		(void)sensor_value_from_float(&val[i], sensor_axes[i]);
	}

	return 0;
}

static const struct sensor_driver_api sitl_imu_api = {
	.sample_fetch = sitl_imu_sample_fetch,
	.channel_get = sitl_imu_channel_get,
};

#define CEREBRI2_SITL_IMU_INIT(inst)                                                              \
	static struct sitl_imu_data sitl_imu_data_##inst;                                        \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, sitl_imu_init, NULL, &sitl_imu_data_##inst, NULL,    \
				     POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,            \
				     &sitl_imu_api)

DT_INST_FOREACH_STATUS_OKAY(CEREBRI2_SITL_IMU_INIT)
