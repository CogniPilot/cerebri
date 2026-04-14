#ifndef RDD2_FLIGHT_MODE_H_
#define RDD2_FLIGHT_MODE_H_

#include <stdint.h>

#include "topic_flatbuffer.h"

#define RDD2_FLIGHT_MODE_CHANNEL_INDEX 5
#define RDD2_FLIGHT_MODE_SWITCH_US     1500

enum rdd2_flight_mode {
	RDD2_FLIGHT_MODE_ACRO = 0,
	RDD2_FLIGHT_MODE_AUTO_LEVEL = 1,
};

enum rdd2_flight_mode rdd2_flight_mode_from_rc(const synapse_topic_RcChannels16_t *rc);
const char *rdd2_flight_mode_name(enum rdd2_flight_mode mode);

#endif
