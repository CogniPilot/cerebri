#ifndef CUBS2_HOST_DEPLOY_IO_H_
#define CUBS2_HOST_DEPLOY_IO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "topic_flatbuffer.h"

int cubs2_host_deploy_io_init(void);

bool cubs2_host_deploy_io_get_mocap(cubs2_mocap_rigid_body_t *mocap);

bool cubs2_host_deploy_io_get_rc(synapse_topic_RcChannels16_t *rc);

void cubs2_host_deploy_io_put_mocap_payload(const uint8_t *buf, size_t len);

void cubs2_host_deploy_io_put_rc(const synapse_topic_RcChannels16_t *rc, bool valid);

#endif /* CUBS2_HOST_DEPLOY_IO_H_ */
