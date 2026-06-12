/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flight_mode.h"

enum cubs2_flight_mode cubs2_flight_mode_from_rc(
	const synapse_topic_RcChannels16_t *rc)
{
	const int32_t *channels;

	if (rc == NULL) {
		return CUBS2_FLIGHT_MODE_ACRO;
	}

	channels = cubs2_topic_rc_channels_data_const(rc);
	if (channels[CUBS2_FLIGHT_MODE_CHANNEL_INDEX] >= CUBS2_FLIGHT_MODE_SWITCH_US) {
		return CUBS2_FLIGHT_MODE_AUTO_LEVEL;
	}

	return CUBS2_FLIGHT_MODE_ACRO;
}

const char *cubs2_flight_mode_name(enum cubs2_flight_mode mode)
{
	switch (mode) {
	case CUBS2_FLIGHT_MODE_ACRO:
		return "ACRO";
	case CUBS2_FLIGHT_MODE_AUTO_LEVEL:
		return "AUTO_LEVEL";
	default:
		return "UNKNOWN";
	}
}
