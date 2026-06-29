#ifndef CUBS2_SITL_UDP_COORDINATOR_H_
#define CUBS2_SITL_UDP_COORDINATOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CUBS2_SITL_INPUT_MAX_SIZE 256U

bool cubs2_sitl_udp_latest_input_get(
	uint8_t *buf, size_t buf_size, size_t *len, uint32_t *generation);

bool cubs2_sitl_udp_publish_input(const uint8_t *buf, size_t len);

#endif
