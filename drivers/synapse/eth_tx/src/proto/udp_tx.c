#include <threads.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>

#include "udp_tx.h"

LOG_MODULE_DECLARE(eth_tx);

#define MY_PORT 4242

int udp_tx_init(struct udp_tx *ctx)
{
	ctx->sock = -1;
	ctx->addr.sin_addr.s_addr = INADDR_ANY;
	ctx->addr.sin_family = AF_INET;
	ctx->addr.sin_port = htons(MY_PORT);

	ctx->sock =
		zsock_socket(((struct sockaddr *)&ctx->addr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);
	if (ctx->sock < 0) {
		LOG_ERR("failed ot create UDP socket: %d", errno);
		return -errno;
	}
	return 0;
}

int udp_tx_fini(struct udp_tx *ctx)
{
	int ret = 0;
	ret = zsock_close(ctx->sock);
	if (ret < 0) {
		LOG_ERR("failed to close socket: %d", ret);
	}
	return ret;
}

int udp_tx_send(struct udp_tx *ctx, const uint8_t *buf, size_t len)
{
	int ret = 0;
	uint32_t addr;
	zsock_inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &addr);
	struct sockaddr_in dest_addr = {
		.sin_addr.s_addr = addr, .sin_family = AF_INET, .sin_port = htons(MY_PORT)};

	ret = zsock_sendto(ctx->sock, buf, len, ZSOCK_MSG_DONTWAIT, (struct sockaddr *)&dest_addr,
			   sizeof(dest_addr));

	if (ret == 0) {
		return -EIO;
	} else if (ret < 0) {
		if (errno == EAGAIN) {
			LOG_INF("timeout");
			ret = 0;
		} else if (errno == EWOULDBLOCK) {
			LOG_INF("would block");
			ret = 0;
		} else {
			LOG_DBG("send error: %d", -errno);
			ret = -errno;
		}
	}
	return ret;
}

// vi: ts=4 sw=4 et
