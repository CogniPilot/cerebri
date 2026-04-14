/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flight_mode.h"

enum rdd2_flight_mode rdd2_flight_mode_from_rc(const synapse_topic_RcChannels16_t *rc)
{
	const int32_t *channels;

	if (rc == NULL) {
		return RDD2_FLIGHT_MODE_ACRO;
	}

	channels = rdd2_topic_rc_channels_data_const(rc);
	if (channels[RDD2_FLIGHT_MODE_CHANNEL_INDEX] >= RDD2_FLIGHT_MODE_SWITCH_US) {
		return RDD2_FLIGHT_MODE_AUTO_LEVEL;
	}

	return RDD2_FLIGHT_MODE_ACRO;
}

const char *rdd2_flight_mode_name(enum rdd2_flight_mode mode)
{
	switch (mode) {
	case RDD2_FLIGHT_MODE_ACRO:
		return "ACRO";
	case RDD2_FLIGHT_MODE_AUTO_LEVEL:
		return "AUTO_LEVEL";
	default:
		return "UNKNOWN";
	}
}
