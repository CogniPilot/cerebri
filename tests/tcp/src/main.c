/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_native_posix_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <errno.h>

#include <zephyr/net/socket.h>
#include <zephyr/net/tftp.h>
#include <zephyr/irq.h>
#include <fcntl.h>


#include "zephyr/../../boards/posix/native_posix/hw_counter.h"
#include "zephyr/../../boards/posix/native_posix/hw_models_top.h"
#include "zephyr/../../boards/posix/native_posix/timer_model.h"


#include <stdio.h>
#define PEER_PORT 4000
#define STACK_SIZE 4096

K_THREAD_STACK_DEFINE(stack_recv, 4096);
K_THREAD_STACK_DEFINE(stack_send, 4096);

static int sock = 0;
static int client_file_descriptor = 0;
static struct k_thread th_recv, th_send;
static int message_count = 1000;

struct pollfd pollfds[1];

#define fatal(msg, ...) { \
		printf("Error: " msg "\n", ##__VA_ARGS__); \
		exit(1); \
	}

static void setblocking(int fd, bool val)
{
	int fl, res;

	fl = fcntl(fd, F_GETFL, 0);
	if (fl == -1) {
		fatal("fcntl(F_GETFL): %d", errno);
	}

	if (val) {
		fl &= ~O_NONBLOCK;
	} else {
		fl |= O_NONBLOCK;
	}

	res = fcntl(fd, F_SETFL, fl);
	if (fl == -1) {
		fatal("fcntl(F_SETFL): %d", errno);
	}
}

void tcp_connect(void) {
    struct sockaddr_in server_address;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("\n Error while creating socket: %d\n", sock);
        exit(1);
    }

    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PEER_PORT);
    int ret = inet_pton(AF_INET, "192.0.2.2", &server_address.sin_addr);
    if (ret < 0) {
        printf("\nInvalid server address/ Address not found: %d\n", ret);
        exit(1);
    }

    // connecting to server
    client_file_descriptor = connect(
        sock, (struct sockaddr*)&server_address,
        sizeof(server_address));
    if (client_file_descriptor < 0) {
        printf("\nConnection failed\n");
        exit(1);
    }
    printf("\nConnected\n");

    pollfds[0].fd = sock;
    pollfds[0].events = POLLIN;

    setblocking(sock, false);
}

void tcp_send(void *id, void *unused, void *unused2) {
    char message[20] = { 0 };
    int i = 0;
    
    while (true) {
        snprintf(message, 20, "%10d hello", i);
        send(sock, message, strlen(message), 0);
        int64_t ticks = k_uptime_ticks();
        printf("send %3d: %10lld: %s\n", strlen(message), ticks, message);
        int res = poll(pollfds, 1, 1000000);
        if (res < 0) {
            printk("poll: %d", res);
            continue;
        }
        char buffer[1024] = { 0 };
        int n_read = recv(sock, buffer, 1024, MSG_DONTWAIT);
        int64_t sim_time = atoi(buffer);
        int64_t up_time = k_uptime_get();
        if (n_read > 1) {
            printf("recv %3d: zephyr %10lld: sim %10lld\n", n_read, up_time, sim_time);
        }
        if (i++ > message_count) break;
    }

    // closing the connected socket
    close(client_file_descriptor);
    printf("\nClosed\n");
    exit(0);
}

void tcp_recv(void *id, void *unused, void *unused2) {
    int i = 0;
    while (true) {
        int res = poll(pollfds, 1, -1);
        if (res < 0) {
            printk("poll: %d", res);
            //k_cpu_idle();
            continue;
        }
        char buffer[1024] = { 0 };
        int n_read = recv(sock, buffer, 1024, MSG_DONTWAIT);\
        int64_t ticks = k_uptime_ticks();
        if (n_read > 1) {
            printf("recv %3d: %10lld: %s\n", n_read, ticks, buffer);
        }
        if (i++ > message_count) break;
    }
}

void tcp_close() {
    // closing the connected socket
    close(client_file_descriptor);
    printf("\nClosed\n");
    exit(0);
}

void main() {
    printf("connecting\n");
    tcp_connect();
    printf("send thread\n");
    //tcp_send(NULL, NULL, NULL);

    k_thread_create(&th_send, &stack_send, STACK_SIZE, tcp_send, NULL, NULL, NULL, 1, 0, K_FOREVER);
    k_thread_name_set(&th_send, "tcp_send");

    //k_thread_create(&th_recv, &stack_recv, STACK_SIZE, tcp_recv, NULL, NULL, NULL, 1, 0, K_FOREVER);
    //k_thread_name_set(&th_recv, "tcp_recv");

    k_thread_start(&th_send);
    //k_thread_start(&th_recv);

    k_thread_join(&th_send, K_FOREVER);
    //k_thread_join(&th_recv, K_FOREVER);

    tcp_close();
    exit(0);
}
