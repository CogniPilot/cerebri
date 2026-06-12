#ifndef CUBS2_FLIGHT_MODE_H_
#define CUBS2_FLIGHT_MODE_H_

#include <stdint.h>

#include "topic_flatbuffer.h"

#define CUBS2_FLIGHT_MODE_CHANNEL_INDEX 5
#define CUBS2_FLIGHT_MODE_SWITCH_US     1500

enum cubs2_flight_mode {
	CUBS2_FLIGHT_MODE_ACRO = 0,
	CUBS2_FLIGHT_MODE_AUTO_LEVEL = 1,
};

enum cubs2_flight_mode cubs2_flight_mode_from_rc(
	const synapse_topic_RcChannels16_t *rc);
const char *cubs2_flight_mode_name(enum cubs2_flight_mode mode);

#endif
