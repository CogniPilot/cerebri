/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <soc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <zephyr/sys/ring_buffer.h>

// mutex locking is not necessary, as this is single threaded
// and all consumers can only run while this process is sleeping

#define RX_BUF_SIZE  8192
#define TX_BUF_SIZE  8192
#define GZ_PORT      4241
#define CEREBRI_PORT 4243

extern struct ring_buf g_tx_buf;
extern struct ring_buf g_rx_buf;
extern pthread_mutex_t g_lock_tx;
extern pthread_mutex_t g_lock_rx;

volatile sig_atomic_t g_shutdown;

struct context {
	const char *module_name;
	int sock;
	pthread_t thread;
	struct sockaddr_in client_addr;
	socklen_t client_addr_len;
};

static struct context g_ctx = {
	.module_name = "dream_sil_native",
	.sock = -1,
	.thread = 0,
};

static void udp_init(struct context *ctx)
{
	printf("%s: sim core running\n", ctx->module_name);

	struct sockaddr_in addr;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(CEREBRI_PORT);
	ctx->sock = socket(((struct sockaddr *)&addr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);

	if (ctx->sock < 0) {
		printf("%s failed to create UDP socket: %d\n", ctx->module_name, errno);
		exit(1);
	}

	int ret = -1;
	while (ret < 0) {
		ret = bind(ctx->sock, (struct sockaddr *)&addr, sizeof(addr));
		printf("%s failed to bind UDP socket: %d\n", ctx->module_name, errno);
		struct timespec request, remaining;
		request.tv_sec = 1;
		request.tv_nsec = 0;
		nanosleep(&request, &remaining);
	}
	printf("%s bound UDP socket\n", ctx->module_name);

	// setup client addr
	uint32_t addr_c;
	inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &addr_c);
	ctx->client_addr.sin_addr.s_addr = addr_c;
	ctx->client_addr.sin_family = AF_INET;
	ctx->client_addr.sin_port = htons(GZ_PORT);
	ctx->client_addr_len = sizeof(ctx->client_addr);
}

static void udp_tx(struct context *ctx)
{
	static uint8_t buf[TX_BUF_SIZE];

	pthread_mutex_lock(&g_lock_tx);
	int len = ring_buf_get(&g_tx_buf, buf, TX_BUF_SIZE);
	pthread_mutex_unlock(&g_lock_tx);

	if (ctx->sock >= 0 && len > 0) {
		sendto(ctx->sock, buf, len, 0, (struct sockaddr *)&ctx->client_addr,
		       ctx->client_addr_len);
	}
}

static void udp_rx(struct context *ctx)
{
	struct pollfd pollfds[] = {
		{ctx->sock, POLLIN | POLLHUP, 0},
	};

	int ret = poll(pollfds, ARRAY_SIZE(pollfds), 1000);

	if (ret == 0) {
		printf("%s no sim data\n", ctx->module_name);
		return;
	} else if (ret < 0) {
		printf("%s poll error: %d\n", ctx->module_name, ret);
		return;
	};

	bool data_ready = false;
	for (size_t i = 0; i < ARRAY_SIZE(pollfds); i++) {
		if (pollfds[i].revents & POLLIN) {
			data_ready = true;
		}
		if (pollfds[i].revents & POLLHUP) {
			// disconnect
			continue;
		}
	}

	if (!data_ready) {
		return;
	}

	// write received data to sim_rx_buf
	static uint8_t buf[RX_BUF_SIZE];
	ret = recvfrom(ctx->sock, buf, sizeof(buf), MSG_DONTWAIT,
		       (struct sockaddr *)&ctx->client_addr, &ctx->client_addr_len);

	if (ret == 0) {
		return;
	} else if (ret < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return;
		} else {
			printf("%s error: %d\n", ctx->module_name, errno);
			return;
		}
	}

	pthread_mutex_lock(&g_lock_rx);
	ring_buf_put(&g_rx_buf, buf, ret);
	pthread_mutex_unlock(&g_lock_rx);
}

static void *native_sim_entry_point(void *p0)
{
	struct context *ctx = p0;
	printf("%s: sim core running\n", ctx->module_name);

	udp_init(ctx);

	// process incoming messages
	while (!g_shutdown) {
		udp_rx(ctx);
		udp_tx(ctx);
	}

	printf("%s exitting\n", ctx->module_name);
	exit(0);
	return 0;
}

static void native_sim_start_task(void)
{
	pthread_create(&g_ctx.thread, NULL, native_sim_entry_point, &g_ctx);
}

static void native_sim_stop_task(void)
{
	g_shutdown = 1;
	pthread_join(g_ctx.thread, NULL);
}

// native tasks
NATIVE_TASK(native_sim_start_task, PRE_BOOT_1, 0);
NATIVE_TASK(native_sim_stop_task, ON_EXIT_PRE, 1);

// vi: ts=4 sw=4 et
