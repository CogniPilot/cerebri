/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cognipilot_sitl_dshot

#include <errno.h>

#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

struct sitl_dshot_config {
	uint8_t channel_count;
};

struct sitl_dshot_data {
	uint64_t last_trigger_ns;
};

static uint64_t sitl_dshot_timestamp_now_ns(void)
{
	return k_cyc_to_ns_floor64(k_cycle_get_64());
}

static int sitl_dshot_init(const struct device *dev)
{
	struct sitl_dshot_data *data = dev->data;

	data->last_trigger_ns = 0U;
	return 0;
}

static void sitl_dshot_data_set(const struct device *dev, unsigned channel, uint16_t throttle,
				bool telemetry)
{
	const struct sitl_dshot_config *config = dev->config;

	ARG_UNUSED(throttle);
	ARG_UNUSED(telemetry);

	if (channel >= config->channel_count) {
		return;
	}
}

static void sitl_dshot_trigger(const struct device *dev)
{
	struct sitl_dshot_data *data = dev->data;

	data->last_trigger_ns = sitl_dshot_timestamp_now_ns();
}

static uint64_t sitl_dshot_last_trigger_ns_get(const struct device *dev)
{
	const struct sitl_dshot_data *data = dev->data;

	return data->last_trigger_ns;
}

static uint8_t sitl_dshot_channel_count(const struct device *dev)
{
	const struct sitl_dshot_config *config = dev->config;

	return config->channel_count;
}

static int sitl_dshot_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(chan);
	return 0;
}

static int sitl_dshot_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	const struct sitl_dshot_config *config = dev->config;

	if (chan != SENSOR_CHAN_RPM) {
		return -EINVAL;
	}

	for (size_t i = 0; i < config->channel_count; i++) {
		val[i].val1 = 0;
		val[i].val2 = 0;
	}

	return 0;
}

static const struct nxp_flexio_dshot_driver_api sitl_dshot_api = {
	.sensor =
		{
			.sample_fetch = sitl_dshot_sample_fetch,
			.channel_get = sitl_dshot_channel_get,
		},
	.data_set = sitl_dshot_data_set,
	.trigger = sitl_dshot_trigger,
	.last_trigger_ns_get = sitl_dshot_last_trigger_ns_get,
	.channel_count = sitl_dshot_channel_count,
};

#define RDD2_SITL_DSHOT_INIT(inst)                                                                 \
	BUILD_ASSERT(DT_INST_PROP(inst, channel_count) == 4,                                       \
		     "rdd2 native_sim expects four motor channels");                               \
	static struct sitl_dshot_data sitl_dshot_data_##inst;                                      \
	static const struct sitl_dshot_config sitl_dshot_config_##inst = {                         \
		.channel_count = DT_INST_PROP(inst, channel_count),                                \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, sitl_dshot_init, NULL, &sitl_dshot_data_##inst,         \
				     &sitl_dshot_config_##inst, POST_KERNEL,                       \
				     CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sitl_dshot_api)

DT_INST_FOREACH_STATUS_OKAY(RDD2_SITL_DSHOT_INIT)
