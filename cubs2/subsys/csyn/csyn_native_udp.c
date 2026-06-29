/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "csyn.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_CUBS2_SITL)
#include "sitl_udp_coordinator.h"
#endif

LOG_MODULE_REGISTER(cubs2_csyn_udp, LOG_LEVEL_INF);

#define CSYN_UDP_MAGIC_0 'C'
#define CSYN_UDP_MAGIC_1 'S'
#define CSYN_UDP_MAGIC_2 'Y'
#define CSYN_UDP_MAGIC_3 'N'
#define CSYN_UDP_HEADER_SIZE 8U

static int g_rx_sock = -1;
static int g_tx_sock = -1;
static struct sockaddr_in g_tx_addr;
static K_THREAD_STACK_DEFINE(g_csyn_udp_stack,
			     CONFIG_CUBS2_CSYN_NATIVE_UDP_THREAD_STACK_SIZE);
static struct k_thread g_csyn_udp_thread;

static uint16_t get_le16(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static void put_le16(uint16_t value, uint8_t *buf)
{
	buf[0] = (uint8_t)(value & 0xffU);
	buf[1] = (uint8_t)((value >> 8) & 0xffU);
}

static int socket_set_nonblocking(int sock)
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

static int socket_init(int *sock, uint16_t bind_port)
{
	struct sockaddr_in addr = {0};
	int rc;

	*sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (*sock < 0) {
		return -errno;
	}

	rc = socket_set_nonblocking(*sock);
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

static int destination_init(struct sockaddr_in *addr)
{
	memset(addr, 0, sizeof(*addr));
	addr->sin_family = AF_INET;
	addr->sin_port = htons(CONFIG_CUBS2_CSYN_NATIVE_UDP_TX_PORT);

	if (inet_pton(AF_INET, CONFIG_CUBS2_CSYN_NATIVE_UDP_HOST, &addr->sin_addr) != 1) {
		return -EINVAL;
	}

	return 0;
}

static void publish_rx_topic(enum cubs2_csyn_topic_id topic, const uint8_t *buf, size_t len)
{
	switch (topic) {
	case CUBS2_CSYN_TOPIC_MOCAP_FRAME:
		cubs2_csyn_publish_mocap_frame(buf, len);
		break;
	case CUBS2_CSYN_TOPIC_MANUAL_CONTROL:
		cubs2_csyn_publish_manual_control(buf, len);
		break;
	case CUBS2_CSYN_TOPIC_SIM_INPUT:
		cubs2_csyn_publish_sim_input(buf, len);
#if defined(CONFIG_CUBS2_SITL)
		(void)cubs2_sitl_udp_publish_input(buf, len);
#endif
		break;
	default:
		(void)cubs2_csyn_topic_publish(topic, buf, len);
		break;
	}
}

static void rx_drain(void)
{
	uint8_t buf[CSYN_UDP_HEADER_SIZE + CUBS2_CSYN_TOPIC_MAX_SIZE];

	while (true) {
		ssize_t len = recv(g_rx_sock, buf, sizeof(buf), 0);
		enum cubs2_csyn_topic_id topic;
		uint16_t payload_len;
		uint16_t topic_raw;

		if (len < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				LOG_WRN("csyn udp rx failed: %d", errno);
			}
			return;
		}

		if ((size_t)len < CSYN_UDP_HEADER_SIZE || buf[0] != CSYN_UDP_MAGIC_0 ||
		    buf[1] != CSYN_UDP_MAGIC_1 || buf[2] != CSYN_UDP_MAGIC_2 ||
		    buf[3] != CSYN_UDP_MAGIC_3) {
			continue;
		}

		topic_raw = get_le16(buf + 4U);
		payload_len = get_le16(buf + 6U);
		if ((size_t)payload_len + CSYN_UDP_HEADER_SIZE != (size_t)len) {
			continue;
		}

		topic = (enum cubs2_csyn_topic_id)topic_raw;
		if (cubs2_csyn_topic_info(topic) == NULL) {
			continue;
		}

		publish_rx_topic(topic, buf + CSYN_UDP_HEADER_SIZE, payload_len);
	}
}

static void tx_topic_if_updated(enum cubs2_csyn_topic_id topic, uint32_t *last_generation)
{
	uint8_t buf[CSYN_UDP_HEADER_SIZE + CUBS2_CSYN_TOPIC_MAX_SIZE];
	size_t len;
	uint32_t generation = cubs2_csyn_topic_generation(topic);

	if (generation == 0U || generation == *last_generation) {
		return;
	}

	if (!cubs2_csyn_topic_copy(topic, buf + CSYN_UDP_HEADER_SIZE,
				  CUBS2_CSYN_TOPIC_MAX_SIZE, &len, NULL)) {
		return;
	}

	buf[0] = CSYN_UDP_MAGIC_0;
	buf[1] = CSYN_UDP_MAGIC_1;
	buf[2] = CSYN_UDP_MAGIC_2;
	buf[3] = CSYN_UDP_MAGIC_3;
	put_le16((uint16_t)topic, buf + 4U);
	put_le16((uint16_t)len, buf + 6U);

	(void)sendto(g_tx_sock, buf, len + CSYN_UDP_HEADER_SIZE, 0,
		     (struct sockaddr *)&g_tx_addr, sizeof(g_tx_addr));
	*last_generation = generation;
}

static void csyn_udp_thread(void *arg0, void *arg1, void *arg2)
{
	static uint32_t last_flight_generation;
	static uint32_t last_motor_generation;
	static uint32_t last_control_generation;

	ARG_UNUSED(arg0);
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);

	while (true) {
		rx_drain();
		tx_topic_if_updated(CUBS2_CSYN_TOPIC_FLIGHT_SNAPSHOT, &last_flight_generation);
		tx_topic_if_updated(CUBS2_CSYN_TOPIC_MOTOR_OUTPUT, &last_motor_generation);
		tx_topic_if_updated(CUBS2_CSYN_TOPIC_CONTROL_OUTPUT, &last_control_generation);
		k_sleep(K_MSEC(1));
	}
}

static int csyn_native_udp_init(void)
{
	int rc;

	rc = socket_init(&g_rx_sock, CONFIG_CUBS2_CSYN_NATIVE_UDP_RX_PORT);
	if (rc != 0) {
		LOG_ERR("csyn udp rx socket init failed: %d", -rc);
		return rc;
	}

	rc = socket_init(&g_tx_sock, 0U);
	if (rc != 0) {
		LOG_ERR("csyn udp tx socket init failed: %d", -rc);
		close(g_rx_sock);
		g_rx_sock = -1;
		return rc;
	}

	rc = destination_init(&g_tx_addr);
	if (rc != 0) {
		LOG_ERR("csyn udp destination init failed");
		return rc;
	}

	k_thread_create(&g_csyn_udp_thread, g_csyn_udp_stack,
			K_THREAD_STACK_SIZEOF(g_csyn_udp_stack), csyn_udp_thread,
			NULL, NULL, NULL, CONFIG_CUBS2_CSYN_NATIVE_UDP_THREAD_PRIORITY,
			0, K_NO_WAIT);
	k_thread_name_set(&g_csyn_udp_thread, "cubs2_csyn_udp");

	LOG_INF("csyn udp rx=%d tx=%d host=%s", CONFIG_CUBS2_CSYN_NATIVE_UDP_RX_PORT,
		CONFIG_CUBS2_CSYN_NATIVE_UDP_TX_PORT, CONFIG_CUBS2_CSYN_NATIVE_UDP_HOST);

	return 0;
}

SYS_INIT(csyn_native_udp_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
