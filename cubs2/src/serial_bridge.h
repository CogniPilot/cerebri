#ifndef CUBS2_SERIAL_BRIDGE_H_
#define CUBS2_SERIAL_BRIDGE_H_

#include "topic_flatbuffer.h"

int cubs2_serial_bridge_init(void);
void cubs2_serial_bridge_send_rc(const synapse_topic_RcChannels16_t *rc);
bool cubs2_serial_bridge_get_mocap(cubs2_mocap_rigid_body_t *rb);

#endif
