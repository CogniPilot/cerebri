/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial output to the wsribunma/ppm_bridge Arduino PPM encoder.
 *
 * Wire format (matches ppm_encoder.ino / ppm_bridge_node.cpp):
 *   57600 8N1
 *   14-byte frame: 0xFF 0xFF | ch0..ch4 (uint16 LE) | checksum (uint16 LE)
 *   checksum = (uint16) sum of the five channel values
 *   channel values are PWM microseconds, nominally 1000..2000
 */
#ifndef CUBS2_HOST_PPM_SERIAL_H_
#define CUBS2_HOST_PPM_SERIAL_H_

#include <stdint.h>

#define PPM_NUM_CHANNELS 5

/* Open the serial port to the PPM bridge at 57600 8N1. Returns fd or -1. */
int ppm_serial_open(const char *device);

/* Send one 14-byte frame. channels[5] are PWM microseconds. Returns 0 on success. */
int ppm_serial_send(int fd, const uint16_t channels[PPM_NUM_CHANNELS]);

void ppm_serial_close(int fd);

#endif /* CUBS2_HOST_PPM_SERIAL_H_ */
