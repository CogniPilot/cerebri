/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "serial_bridge.h"
#include "csyn.h"
#include "control_io.h"
#include "topic_shell.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <stdint.h>

#if defined(CONFIG_BOARD_NATIVE_SIM) && defined(CONFIG_CUBS2_SERIAL_BRIDGE_PPM_ARDUINO)
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#endif

#if defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
LOG_MODULE_REGISTER(serial_bridge, LOG_LEVEL_NONE);
#else
LOG_MODULE_REGISTER(serial_bridge, LOG_LEVEL_INF);
#endif

#define UART_DEVICE_NODE DT_NODELABEL(lpuart2)
#define SERIAL_BRIDGE_HAS_UART DT_NODE_HAS_STATUS(UART_DEVICE_NODE, okay)
#if defined(CONFIG_BOARD_NATIVE_SIM) && defined(CONFIG_CUBS2_SERIAL_BRIDGE_PPM_ARDUINO)
#define SERIAL_BRIDGE_HAS_HOST_PPM 1
#else
#define SERIAL_BRIDGE_HAS_HOST_PPM 0
#endif
#define BRIDGE_THREAD_STACK_SIZE 2048
#define BRIDGE_THREAD_PRIORITY 5

#define SYNC_BYTE_1 0x55
#define SYNC_BYTE_2 0xAA
#define PPM_SYNC_BYTE 0xFFU
#define PPM_CHANNEL_COUNT 5U

enum msg_type {
	MSG_TYPE_MOCAP = 0x01,
	MSG_TYPE_RC = 0x02,
};

struct bridge_ctx {
	const struct device *uart;
#if SERIAL_BRIDGE_HAS_HOST_PPM
	int host_ppm_fd;
	const char *host_ppm_device;
	char host_ack_buf[128];
	size_t host_ack_len;
#endif
	uint8_t rx_buf[CUBS2_TOPIC_FB_MOCAP_FRAME_MAX_SIZE];
	size_t rx_idx;
	enum {
		STATE_SYNC1,
		STATE_SYNC2,
		STATE_TYPE,
		STATE_LEN_LO,
		STATE_LEN_HI,
		STATE_PAYLOAD,
		STATE_CHECKSUM
	} state;
	uint8_t type;
	uint16_t len;
	uint8_t checksum;
};

static struct bridge_ctx g_bridge_ctx;
static cubs2_mocap_rigid_body_t g_latest_mocap;
static struct k_mutex g_mocap_mutex;

#if SERIAL_BRIDGE_HAS_HOST_PPM
static int host_ppm_serial_open(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		LOG_ERR("failed to open Arduino PPM serial device %s: %s", device, strerror(errno));
		return -errno;
	}

	if (isatty(fd)) {
		struct termios tty;

		memset(&tty, 0, sizeof(tty));
		if (tcgetattr(fd, &tty) != 0) {
			int rc = -errno;

			LOG_ERR("tcgetattr failed for %s: %s", device, strerror(errno));
			close(fd);
			return rc;
		}

		cfsetospeed(&tty, B57600);
		cfsetispeed(&tty, B57600);

		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
		tty.c_cflag |= (CLOCAL | CREAD);
		tty.c_cflag &= ~(PARENB | PARODD);
		tty.c_cflag &= ~CSTOPB;
#if defined(CRTSCTS)
		tty.c_cflag &= ~CRTSCTS;
#endif
		tty.c_lflag = 0;
		tty.c_oflag = 0;
		tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
		tty.c_cc[VMIN] = 0;
		tty.c_cc[VTIME] = 0;

		if (tcsetattr(fd, TCSANOW, &tty) != 0) {
			int rc = -errno;

			LOG_ERR("tcsetattr failed for %s: %s", device, strerror(errno));
			close(fd);
			return rc;
		}

		tcflush(fd, TCIOFLUSH);
	} else {
		LOG_WRN("%s is not a tty; writing raw Arduino PPM frames", device);
	}

	LOG_INF("Arduino PPM serial output opened on %s @ 57600", device);
	return fd;
}

static int host_ppm_write_all(int fd, const uint8_t *buf, size_t len)
{
	size_t off = 0U;

	while (off < len) {
		ssize_t n = write(fd, buf + off, len - off);

		if (n < 0) {
			if (errno == EINTR || errno == EAGAIN) {
				continue;
			}
			return -errno;
		}

		off += (size_t)n;
	}

	return 0;
}

static void host_ppm_poll_ack(struct bridge_ctx *ctx)
{
	uint8_t buf[64];

	while (true) {
		ssize_t n = read(ctx->host_ppm_fd, buf, sizeof(buf));

		if (n < 0) {
			if (errno == EINTR) {
				continue;
			}
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				return;
			}
			LOG_ERR("Arduino PPM serial read failed on %s: %s", ctx->host_ppm_device,
				strerror(errno));
			return;
		}

		if (n == 0) {
			return;
		}

		for (ssize_t i = 0; i < n; i++) {
			char c = (char)buf[i];

			if (c == '\r') {
				continue;
			}

			if (c == '\n') {
				if (ctx->host_ack_len > 0U) {
					ctx->host_ack_buf[ctx->host_ack_len] = '\0';
					LOG_INF("Arduino serial RX: %s", ctx->host_ack_buf);
					ctx->host_ack_len = 0U;
				}
				continue;
			}

			if (ctx->host_ack_len < (sizeof(ctx->host_ack_buf) - 1U)) {
				ctx->host_ack_buf[ctx->host_ack_len++] = c;
			} else {
				ctx->host_ack_buf[ctx->host_ack_len] = '\0';
				LOG_WRN("Arduino serial RX line too long, dropping partial: %s",
					ctx->host_ack_buf);
				ctx->host_ack_len = 0U;
			}
		}
	}
}
#endif

#if SERIAL_BRIDGE_HAS_UART
K_THREAD_STACK_DEFINE(bridge_thread_stack, BRIDGE_THREAD_STACK_SIZE);
struct k_thread bridge_thread_data;

static void bridge_rx_handle_mocap(const uint8_t *buf, size_t len)
{
	cubs2_mocap_rigid_body_t rb;
	if (cubs2_topic_fb_unpack_mocap_frame(buf, len, &rb)) {
		k_mutex_lock(&g_mocap_mutex, K_FOREVER);
		g_latest_mocap = rb;
		k_mutex_unlock(&g_mocap_mutex);
		cubs2_control_input_trigger();
	}
}

static void bridge_thread(void *p1, void *p2, void *p3)
{
	struct bridge_ctx *ctx = &g_bridge_ctx;
	uint8_t c;

	while (true) {
		if (uart_poll_in(ctx->uart, &c) == 0) {
			switch (ctx->state) {
			case STATE_SYNC1:
				if (c == SYNC_BYTE_1) {
					ctx->state = STATE_SYNC2;
					ctx->checksum = c;
				}
				break;
			case STATE_SYNC2:
				if (c == SYNC_BYTE_2) {
					ctx->state = STATE_TYPE;
					ctx->checksum ^= c;
				} else {
					ctx->state = STATE_SYNC1;
				}
				break;
			case STATE_TYPE:
				ctx->type = c;
				ctx->checksum ^= c;
				ctx->state = STATE_LEN_LO;
				break;
			case STATE_LEN_LO:
				ctx->len = c;
				ctx->checksum ^= c;
				ctx->state = STATE_LEN_HI;
				break;
			case STATE_LEN_HI:
				ctx->len |= (uint16_t)c << 8;
				ctx->checksum ^= c;
				if (ctx->len > sizeof(ctx->rx_buf)) {
					ctx->state = STATE_SYNC1;
				} else {
					ctx->rx_idx = 0;
					ctx->state = STATE_PAYLOAD;
				}
				break;
			case STATE_PAYLOAD:
				ctx->rx_buf[ctx->rx_idx++] = c;
				ctx->checksum ^= c;
				if (ctx->rx_idx >= ctx->len) {
					ctx->state = STATE_CHECKSUM;
				}
				break;
			case STATE_CHECKSUM:
				if (c == ctx->checksum) {
					if (ctx->type == MSG_TYPE_MOCAP) {
						bridge_rx_handle_mocap(ctx->rx_buf, ctx->len);
					}
				}
				ctx->state = STATE_SYNC1;
				break;
			}
		} else {
			k_sleep(K_MSEC(1));
		}
	}
}
#endif

int cubs2_serial_bridge_init(void)
{
	struct bridge_ctx *ctx = &g_bridge_ctx;

	k_mutex_init(&g_mocap_mutex);

#if SERIAL_BRIDGE_HAS_HOST_PPM
	ctx->host_ppm_device = getenv("CUBS2_PPM_DEVICE");
	if (ctx->host_ppm_device == NULL || ctx->host_ppm_device[0] == '\0') {
		ctx->host_ppm_device = "/dev/ttyACM0";
	}
	ctx->host_ppm_fd = host_ppm_serial_open(ctx->host_ppm_device);
	if (ctx->host_ppm_fd >= 0) {
		host_ppm_poll_ack(ctx);
	}
#endif

#if SERIAL_BRIDGE_HAS_UART
	ctx->uart = DEVICE_DT_GET(UART_DEVICE_NODE);
	if (!device_is_ready(ctx->uart)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	ctx->state = STATE_SYNC1;

	k_thread_create(&bridge_thread_data, bridge_thread_stack,
			K_THREAD_STACK_SIZEOF(bridge_thread_stack),
			bridge_thread, NULL, NULL, NULL,
			BRIDGE_THREAD_PRIORITY, 0, K_NO_WAIT);
#else
	ctx->uart = NULL;
#if !SERIAL_BRIDGE_HAS_HOST_PPM
	LOG_INF("UART serial bridge disabled: no lpuart2 device");
#endif
#endif

	return 0;
}

bool cubs2_serial_bridge_get_mocap(cubs2_mocap_rigid_body_t *rb)
{
	k_mutex_lock(&g_mocap_mutex, K_FOREVER);
	*rb = g_latest_mocap;
	k_mutex_unlock(&g_mocap_mutex);
	return rb->valid;
}

#if defined(CONFIG_CUBS2_SERIAL_BRIDGE_PPM_ARDUINO)
static uint16_t ppm_channel_from_rc(int32_t value)
{
	if (value < 0) {
		return 0U;
	}
	if (value > UINT16_MAX) {
		return UINT16_MAX;
	}
	return (uint16_t)value;
}

static void uart_write_le16(const struct device *uart, uint16_t value)
{
	uart_poll_out(uart, (uint8_t)(value & 0xffU));
	uart_poll_out(uart, (uint8_t)((value >> 8) & 0xffU));
}
#endif

void cubs2_serial_bridge_send_rc(const synapse_topic_RcChannels16_t *rc)
{
	struct bridge_ctx *ctx = &g_bridge_ctx;
	static int64_t s_last_no_uart_log_ms = -1000;
#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	static int64_t s_last_send_log_ms = -1000;
#endif
	int64_t now_ms;

#if defined(CONFIG_CUBS2_SERIAL_BRIDGE_CSYN_OUTPUT)
	ARG_UNUSED(ctx);
	ARG_UNUSED(s_last_no_uart_log_ms);
#if defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	ARG_UNUSED(now_ms);
#endif
	cubs2_csyn_publish_control_output(rc);

#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	now_ms = k_uptime_get();
	if ((now_ms - s_last_send_log_ms) >= 1000) {
		LOG_INF("serial bridge output routed to csyn control_output");
		s_last_send_log_ms = now_ms;
	}
#endif
	return;
#endif

#if defined(CONFIG_CUBS2_SERIAL_BRIDGE_PPM_ARDUINO)
	const int32_t *channels_in = cubs2_topic_rc_channels_data_const(rc);
	uint16_t channels[PPM_CHANNEL_COUNT];
	uint16_t checksum = 0U;

	for (size_t i = 0U; i < PPM_CHANNEL_COUNT; i++) {
		channels[i] = ppm_channel_from_rc(channels_in[i]);
		checksum = (uint16_t)(checksum + channels[i]);
	}

#if SERIAL_BRIDGE_HAS_HOST_PPM
	if (ctx->host_ppm_fd >= 0) {
		uint8_t frame[14];
		int rc_write;

		frame[0] = PPM_SYNC_BYTE;
		frame[1] = PPM_SYNC_BYTE;
		for (size_t i = 0U; i < PPM_CHANNEL_COUNT; i++) {
			frame[2U + (i * 2U)] = (uint8_t)(channels[i] & 0xffU);
			frame[3U + (i * 2U)] = (uint8_t)((channels[i] >> 8) & 0xffU);
		}
		frame[12] = (uint8_t)(checksum & 0xffU);
		frame[13] = (uint8_t)((checksum >> 8) & 0xffU);

		rc_write = host_ppm_write_all(ctx->host_ppm_fd, frame, sizeof(frame));
		if (rc_write != 0) {
			now_ms = k_uptime_get();
			if ((now_ms - s_last_no_uart_log_ms) >= 1000) {
				LOG_ERR("Arduino PPM frame write failed on %s: %s",
					ctx->host_ppm_device, strerror(-rc_write));
				s_last_no_uart_log_ms = now_ms;
			}
			return;
		}
		host_ppm_poll_ack(ctx);
	} else
#endif
	if (ctx->uart != NULL) {
		uart_poll_out(ctx->uart, PPM_SYNC_BYTE);
		uart_poll_out(ctx->uart, PPM_SYNC_BYTE);
		for (size_t i = 0U; i < PPM_CHANNEL_COUNT; i++) {
			uart_write_le16(ctx->uart, channels[i]);
		}
		uart_write_le16(ctx->uart, checksum);
	} else {
		now_ms = k_uptime_get();
		if ((now_ms - s_last_no_uart_log_ms) >= 1000) {
			LOG_WRN("serial bridge RC frame not sent: no UART or host PPM device");
			s_last_no_uart_log_ms = now_ms;
		}
		return;
	}

#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	now_ms = k_uptime_get();
	if ((now_ms - s_last_send_log_ms) >= 1000) {
#if SERIAL_BRIDGE_HAS_HOST_PPM
		if (ctx->host_ppm_fd >= 0) {
			LOG_INF("serial bridge sent Arduino PPM frame to %s ch=[%u %u %u %u %u] checksum=%u",
				ctx->host_ppm_device, (unsigned int)channels[0],
				(unsigned int)channels[1], (unsigned int)channels[2],
				(unsigned int)channels[3], (unsigned int)channels[4],
				(unsigned int)checksum);
		} else
#endif
		{
			LOG_INF("serial bridge sent Arduino PPM frame ch=[%u %u %u %u %u] checksum=%u",
			(unsigned int)channels[0], (unsigned int)channels[1],
			(unsigned int)channels[2], (unsigned int)channels[3],
			(unsigned int)channels[4], (unsigned int)checksum);
		}
		s_last_send_log_ms = now_ms;
	}
#endif
#else
	if (ctx->uart == NULL) {
		now_ms = k_uptime_get();
		if ((now_ms - s_last_no_uart_log_ms) >= 1000) {
			LOG_WRN("serial bridge RC frame not sent: no UART device");
			s_last_no_uart_log_ms = now_ms;
		}
		return;
	}

	uint8_t header[5];
	uint8_t checksum = 0;
	const uint8_t *payload = (const uint8_t *)rc;
	size_t len = sizeof(*rc);

	header[0] = SYNC_BYTE_1;
	header[1] = SYNC_BYTE_2;
	header[2] = MSG_TYPE_RC;
	header[3] = (uint8_t)(len & 0xff);
	header[4] = (uint8_t)((len >> 8) & 0xff);

	for (int i = 0; i < 5; i++) {
		uart_poll_out(ctx->uart, header[i]);
		checksum ^= header[i];
	}

	for (size_t i = 0; i < len; i++) {
		uart_poll_out(ctx->uart, payload[i]);
		checksum ^= payload[i];
	}

	uart_poll_out(ctx->uart, checksum);

#if !defined(CONFIG_CUBS2_SERIAL_BRIDGE_QUIET)
	now_ms = k_uptime_get();
	if ((now_ms - s_last_send_log_ms) >= 1000) {
		LOG_INF("serial bridge sent internal RC frame len=%u checksum=%u",
			(unsigned int)len, (unsigned int)checksum);
		s_last_send_log_ms = now_ms;
	}
#endif
#endif
}
