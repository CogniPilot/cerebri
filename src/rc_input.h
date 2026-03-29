#ifndef CEREBRI2_RC_INPUT_H_
#define CEREBRI2_RC_INPUT_H_

#include <stdint.h>

#include <zephyr/device.h>

#include "topic_messages.h"

#define CEREBRI2_RC_INPUT_EVENT_LINK_QUALITY 0x1000
#define CEREBRI2_RC_INPUT_EVENT_VALID        0x1001

void cerebri2_rc_input_init(void);
struct rc_frame cerebri2_rc_input_snapshot(void);
uint8_t cerebri2_rc_input_link_quality_get(const struct device *dev);

#endif
