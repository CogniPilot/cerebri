/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rc_input.h"
#include "sitl_flatbuffer.h"
#include "sitl_udp_coordinator.h"
#include "topic_bus.h"
#include "topic_flatbuffer.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rdd2_sitl_udp, LOG_LEVEL_INF);

struct sitl_input_store {
	uint8_t slots[2][RDD2_SITL_INPUT_MAX_SIZE];
	uint16_t lengths[2];
	atomic_t generation;
};

static struct sitl_input_store g_sitl_input_store;
static int g_sitl_rx_sock = -1;
static int g_sitl_tx_sock = -1;
static struct sockaddr_in g_sitl_tx_flight_addr;
static struct sockaddr_in g_sitl_tx_motor_addr;
static K_THREAD_STACK_DEFINE(g_sitl_thread_stack, CONFIG_RDD2_SITL_THREAD_STACK_SIZE);
static struct k_thread g_sitl_thread;

static int sitl_socket_set_nonblocking(int sock)
{
	int flags = fcntl(sock, F_GETFL, 0);

	if (flags < 0) {
		return -errno;
	}

	if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
		return -errno;
	}

	return 0;
}

static int sitl_socket_init(int *sock, uint16_t bind_port)
{
	struct sockaddr_in addr = {0};
	int rc;

	*sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (*sock < 0) {
		return -errno;
	}

	rc = sitl_socket_set_nonblocking(*sock);
	if (rc != 0) {
		close(*sock);
		*sock = -1;
		return rc;
	}

	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(bind_port);

	if (bind_port != 0U && bind(*sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		rc = -errno;
		close(*sock);
		*sock = -1;
		return rc;
	}

	return 0;
}

static int sitl_destination_init(struct sockaddr_in *addr, uint16_t port)
{
	memset(addr, 0, sizeof(*addr));
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);

	if (inet_pton(AF_INET, CONFIG_RDD2_SITL_HOST, &addr->sin_addr) != 1) {
		return -EINVAL;
	}

	return 0;
}

static void sitl_input_store_publish(const uint8_t *buf, size_t len)
{
	uint32_t next_generation = (uint32_t)atomic_get(&g_sitl_input_store.generation) + 1U;
	uint32_t slot = next_generation & 1U;

	memcpy(g_sitl_input_store.slots[slot], buf, len);
	g_sitl_input_store.lengths[slot] = (uint16_t)len;
	atomic_set(&g_sitl_input_store.generation, (atomic_val_t)next_generation);
}

bool rdd2_sitl_udp_latest_input_get(uint8_t *buf, size_t buf_size, size_t *len,
				    uint32_t *generation)
{
	uint32_t generation_start;
	uint32_t generation_end;
	uint32_t slot;
	uint16_t length;

	if (buf == NULL || len == NULL || generation == NULL) {
		return false;
	}

	do {
		generation_start = (uint32_t)atomic_get(&g_sitl_input_store.generation);
		if (generation_start == 0U) {
			return false;
		}

		slot = generation_start & 1U;
		length = g_sitl_input_store.lengths[slot];
		if (length == 0U || length > buf_size) {
			return false;
		}

		memcpy(buf, g_sitl_input_store.slots[slot], length);
		generation_end = (uint32_t)atomic_get(&g_sitl_input_store.generation);
	} while (generation_start != generation_end);

	*len = length;
	*generation = generation_start;
	return true;
}

static void sitl_report_rc_input(const synapse_topic_RcChannels16_t *rc, uint8_t rc_link_quality,
				 bool rc_valid)
{
	const struct device *const rc_dev = DEVICE_DT_GET(DT_ALIAS(rc));
	const int32_t *channels = rdd2_topic_rc_channels_data_const(rc);

	if (!device_is_ready(rc_dev)) {
		return;
	}

	for (size_t i = 0; i < 16U; i++) {
		(void)input_report_abs(rc_dev, (uint16_t)(i + 1U), channels[i], false, K_FOREVER);
	}

	(void)input_report(rc_dev, INPUT_EV_MSC, RDD2_RC_INPUT_EVENT_LINK_QUALITY, rc_link_quality,
			   false, K_FOREVER);
	(void)input_report(rc_dev, INPUT_EV_MSC, RDD2_RC_INPUT_EVENT_VALID, rc_valid ? 1 : 0, true,
			   K_FOREVER);
}

static void sitl_rx_drain(void)
{
	struct sockaddr_in source_addr;
	socklen_t source_addr_len = sizeof(source_addr);
	uint8_t buf[RDD2_SITL_INPUT_MAX_SIZE];

	while (true) {
		synapse_topic_RcChannels16_t rc;
		uint8_t rc_link_quality;
		bool rc_valid;
		ssize_t len = recvfrom(g_sitl_rx_sock, buf, sizeof(buf), 0,
				       (struct sockaddr *)&source_addr, &source_addr_len);

		if (len < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				LOG_WRN("sitl rx failed: %d", errno);
			}
			break;
		}

		if (!rdd2_sitl_fb_unpack_input(buf, (size_t)len, NULL, NULL, &rc, &rc_link_quality,
					       &rc_valid, NULL)) {
			source_addr_len = sizeof(source_addr);
			continue;
		}

		sitl_input_store_publish(buf, (size_t)len);
		sitl_report_rc_input(&rc, rc_link_quality, rc_valid);
		source_addr_len = sizeof(source_addr);
	}
}

static void sitl_send_flight_state_if_updated(void)
{
	static uint32_t last_generation;
	uint32_t generation = rdd2_topic_flight_state_generation();
	uint8_t buf[RDD2_TOPIC_FB_FLIGHT_STATE_SIZE];
	size_t len;

	if (generation == 0U || generation == last_generation) {
		return;
	}

	if (!rdd2_topic_flight_state_copy_blob(buf, sizeof(buf), &len)) {
		return;
	}

	(void)sendto(g_sitl_tx_sock, buf, len, 0, (struct sockaddr *)&g_sitl_tx_flight_addr,
		     sizeof(g_sitl_tx_flight_addr));
	last_generation = generation;
}

static void sitl_send_motor_output_if_updated(void)
{
	static uint32_t last_generation;
	uint32_t generation = rdd2_topic_motor_output_generation();
	uint8_t buf[RDD2_TOPIC_FB_MOTOR_OUTPUT_SIZE];
	size_t len;

	if (generation == 0U || generation == last_generation) {
		return;
	}

	if (!rdd2_topic_motor_output_copy_blob(buf, sizeof(buf), &len)) {
		return;
	}

	(void)sendto(g_sitl_tx_sock, buf, len, 0, (struct sockaddr *)&g_sitl_tx_motor_addr,
		     sizeof(g_sitl_tx_motor_addr));
	last_generation = generation;
}

static void sitl_transport_thread(void *arg0, void *arg1, void *arg2)
{
	ARG_UNUSED(arg0);
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);

	while (true) {
		sitl_rx_drain();
		sitl_send_flight_state_if_updated();
		sitl_send_motor_output_if_updated();
		k_sleep(K_MSEC(1));
	}
}

static int rdd2_sitl_init(void)
{
	int rc;

	atomic_set(&g_sitl_input_store.generation, 0);

	rc = sitl_socket_init(&g_sitl_rx_sock, CONFIG_RDD2_SITL_RX_PORT);
	if (rc != 0) {
		LOG_ERR("sitl rx socket init failed: %d", -rc);
		return rc;
	}

	rc = sitl_socket_init(&g_sitl_tx_sock, 0U);
	if (rc != 0) {
		LOG_ERR("sitl tx socket init failed: %d", -rc);
		close(g_sitl_rx_sock);
		g_sitl_rx_sock = -1;
		return rc;
	}

	rc = sitl_destination_init(&g_sitl_tx_flight_addr,
				   CONFIG_RDD2_SITL_TX_FLIGHT_SNAPSHOT_PORT);
	if (rc != 0) {
		LOG_ERR("sitl flight destination init failed");
		return rc;
	}

	rc = sitl_destination_init(&g_sitl_tx_motor_addr, CONFIG_RDD2_SITL_TX_MOTOR_OUTPUT_PORT);
	if (rc != 0) {
		LOG_ERR("sitl motor destination init failed");
		return rc;
	}

	k_thread_create(&g_sitl_thread, g_sitl_thread_stack,
			K_THREAD_STACK_SIZEOF(g_sitl_thread_stack), sitl_transport_thread, NULL,
			NULL, NULL, CONFIG_RDD2_SITL_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&g_sitl_thread, "rdd2_sitl");

	LOG_INF("sitl udp rx=%d tx_flight=%d tx_motor=%d host=%s", CONFIG_RDD2_SITL_RX_PORT,
		CONFIG_RDD2_SITL_TX_FLIGHT_SNAPSHOT_PORT, CONFIG_RDD2_SITL_TX_MOTOR_OUTPUT_PORT,
		CONFIG_RDD2_SITL_HOST);

	return 0;
}

SYS_INIT(rdd2_sitl_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
