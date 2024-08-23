#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>

#include "udp_rx.h"

LOG_MODULE_DECLARE(eth_rx);

#define MY_PORT 4242

int udp_rx_init(struct udp_rx *ctx)
{
	ctx->addr.sin_addr.s_addr = INADDR_ANY;
	ctx->addr.sin_family = AF_INET;
	ctx->addr.sin_port = htons(MY_PORT);
	ctx->sock =
		zsock_socket(((struct sockaddr *)&ctx->addr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);
	if (ctx->sock < 0) {
		LOG_ERR("failed ot create UDP socket: %d", errno);
		return -errno;
	}

	int ret = 0;
	ret = zsock_bind(ctx->sock, (struct sockaddr *)&ctx->addr, sizeof(ctx->addr));
	if (ret < 0) {
		LOG_ERR("failed to bind UDP socket: %d", errno);
		return -errno;
	}

	return 0;
}

int udp_rx_fini(struct udp_rx *ctx)
{
	int ret = 0;
	ret = zsock_close(ctx->sock);
	if (ret < 0) {
		LOG_ERR("failed to close socket: %d", ret);
	}
	return ret;
}

int udp_rx_receive(struct udp_rx *ctx)
{
	uint32_t addr;
	zsock_inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &addr);
	struct sockaddr_in client_addr = {
		.sin_addr.s_addr = addr, .sin_family = AF_INET, .sin_port = htons(MY_PORT)};
	socklen_t client_addr_len = sizeof(client_addr);
	int ret = 0;

	struct zsock_pollfd fds[] = {
		{ctx->sock, ZSOCK_POLLIN | ZSOCK_POLLHUP, 0},
	};

	ret = zsock_poll(fds, ARRAY_SIZE(fds), 1000);

	if (ret == 0) {
		return 0;
	} else if (ret < 0) {
		LOG_ERR("poll failed: %d", ret);
		return ret;
	}

	bool data_ready = false;
	for (size_t i = 0; i < ARRAY_SIZE(fds); i++) {
		if (fds[i].revents & ZSOCK_POLLIN) {
			data_ready = true;
		}
		if (fds[i].revents & ZSOCK_POLLHUP) {
			// socket closed
		}
	}

	if (!data_ready) {
		return 0;
	}

	ret = zsock_recvfrom(ctx->sock, ctx->rx_buf, sizeof(ctx->rx_buf), ZSOCK_MSG_DONTWAIT,
			     (struct sockaddr *)&client_addr, &client_addr_len);

	if (ret == 0) {
		return -EIO;
	} else if (ret < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			ret = 0;
		} else {
			ret = -errno;
		}
	}
	return ret;
}

// vi: ts=4 sw=4 et
