#ifndef RDD2_RC_INPUT_H_
#define RDD2_RC_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

#include "topic_flatbuffer.h"

#define RDD2_RC_INPUT_EVENT_LINK_QUALITY 0x1000
#define RDD2_RC_INPUT_EVENT_VALID        0x1001

void rdd2_rc_input_init(void);
void rdd2_rc_input_latest_get(synapse_topic_RcChannels16_t *rc, int64_t *stamp_ms, bool *valid);
uint8_t rdd2_rc_input_link_quality_get(const struct device *dev);

#endif
