/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "serial_bridge.h"
#include "control_io.h"
#include "topic_shell.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(serial_bridge, LOG_LEVEL_INF);

#define UART_DEVICE_NODE DT_NODELABEL(lpuart2)
#define SERIAL_BRIDGE_HAS_UART DT_NODE_HAS_STATUS(UART_DEVICE_NODE, okay)
#define BRIDGE_THREAD_STACK_SIZE 2048
#define BRIDGE_THREAD_PRIORITY 5

#define SYNC_BYTE_1 0x55
#define SYNC_BYTE_2 0xAA

enum msg_type {
	MSG_TYPE_MOCAP = 0x01,
	MSG_TYPE_RC = 0x02,
};

struct bridge_ctx {
	const struct device *uart;
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

#if SERIAL_BRIDGE_HAS_UART
K_THREAD_STACK_DEFINE(bridge_thread_stack, BRIDGE_THREAD_STACK_SIZE);
struct k_thread bridge_thread_data;
#endif

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

#if SERIAL_BRIDGE_HAS_UART
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
	LOG_INF("UART serial bridge disabled: no lpuart2 device");
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

void cubs2_serial_bridge_send_rc(const synapse_topic_RcChannels16_t *rc)
{
	struct bridge_ctx *ctx = &g_bridge_ctx;

	if (ctx->uart == NULL) {
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
}
