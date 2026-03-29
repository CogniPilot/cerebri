#ifndef CEREBRI2_SITL_UDP_COORDINATOR_H_
#define CEREBRI2_SITL_UDP_COORDINATOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CEREBRI2_SITL_INPUT_MAX_SIZE 256U

bool cerebri2_sitl_udp_latest_input_get(
	uint8_t *buf, size_t buf_size, size_t *len, uint32_t *generation);

#endif
