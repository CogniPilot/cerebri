/*
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ppm_serial.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int ppm_serial_open(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		fprintf(stderr, "ppm_serial: open(%s): %s\n", device, strerror(errno));
		return -1;
	}

	if (!isatty(fd)) {
		/* Non-tty sink (file, pipe, /dev/null): skip line discipline setup.
		 * Useful for logging/replay and for testing without hardware. */
		fprintf(stderr, "ppm_serial: %s is not a tty; sending raw frames\n", device);
		return fd;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "ppm_serial: tcgetattr: %s\n", strerror(errno));
		close(fd);
		return -1;
	}

	cfsetospeed(&tty, B57600);
	cfsetispeed(&tty, B57600);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 data bits */
	tty.c_cflag |= (CLOCAL | CREAD);            /* enable receiver, ignore modem ctrl */
	tty.c_cflag &= ~(PARENB | PARODD);          /* no parity */
	tty.c_cflag &= ~CSTOPB;                     /* one stop bit */
	tty.c_cflag &= ~CRTSCTS;                    /* no hw flow control */

	/* raw mode */
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "ppm_serial: tcsetattr: %s\n", strerror(errno));
		close(fd);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);
	return fd;
}

int ppm_serial_send(int fd, const uint16_t channels[PPM_NUM_CHANNELS])
{
	uint8_t frame[14];
	uint16_t cksum = 0U;

	frame[0] = 0xFFU;
	frame[1] = 0xFFU;
	for (int i = 0; i < PPM_NUM_CHANNELS; i++) {
		frame[2 + i * 2] = (uint8_t)(channels[i] & 0xFFU);
		frame[2 + i * 2 + 1] = (uint8_t)((channels[i] >> 8) & 0xFFU);
		cksum = (uint16_t)(cksum + channels[i]);
	}
	frame[12] = (uint8_t)(cksum & 0xFFU);
	frame[13] = (uint8_t)((cksum >> 8) & 0xFFU);

	size_t off = 0;
	while (off < sizeof(frame)) {
		ssize_t n = write(fd, frame + off, sizeof(frame) - off);
		if (n < 0) {
			if (errno == EAGAIN || errno == EINTR) {
				continue;
			}
			return -1;
		}
		off += (size_t)n;
	}
	return 0;
}

void ppm_serial_close(int fd)
{
	if (fd >= 0) {
		close(fd);
	}
}
