/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>

void nxp_flexio_dshot_data_set(const struct device *dev, unsigned channel, uint16_t throttle,
			       bool telemetry)
{
	const struct nxp_flexio_dshot_driver_api *api =
		(const struct nxp_flexio_dshot_driver_api *)dev->api;

	if (api == NULL || api->data_set == NULL) {
		return;
	}

	api->data_set(dev, channel, throttle, telemetry);
}

void nxp_flexio_dshot_trigger(const struct device *dev)
{
	const struct nxp_flexio_dshot_driver_api *api =
		(const struct nxp_flexio_dshot_driver_api *)dev->api;

	if (api == NULL || api->trigger == NULL) {
		return;
	}

	api->trigger(dev);
}

uint64_t nxp_flexio_dshot_last_trigger_ns_get(const struct device *dev)
{
	const struct nxp_flexio_dshot_driver_api *api =
		(const struct nxp_flexio_dshot_driver_api *)dev->api;

	if (api == NULL || api->last_trigger_ns_get == NULL) {
		return 0U;
	}

	return api->last_trigger_ns_get(dev);
}

uint8_t nxp_flexio_dshot_channel_count(const struct device *dev)
{
	const struct nxp_flexio_dshot_driver_api *api =
		(const struct nxp_flexio_dshot_driver_api *)dev->api;

	if (api == NULL || api->channel_count == NULL) {
		return 0U;
	}

	return api->channel_count(dev);
}
