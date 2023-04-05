/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "clock.pb.h"
#include <zephyr/kernel.h>


#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <time.h>
#include <soc.h>
#include <fcntl.h>

#include <synapse_zbus/channels.h>

#include "synapse_tinyframe/SynapseTopics.h"
#include "synapse_tinyframe/TinyFrame.h"
#include "synapse_tinyframe/utils.h"

#define MY_STACK_SIZE 500
#define MY_PRIORITY -10
#define BIND_PORT 4241
#define RX_BUF_SIZE 1024

int64_t connect_time = 0;
Clock sim_clock = Clock_init_default;
static int client = 0;

enum tf_comm_t {
    TF_COMM_UART_0 = 0,
    TF_COMM_UART_1 = 1,
    TF_COMM_ETHR_0 = 10,
    TF_COMM_SIM_0 = 30
};

#define PUB_SIM_MESSAGES(TOPIC, MSG)                        \
    {                                                       \
        msg_##MSG##_t data;                                 \
        while (queue_##TOPIC.tryPop(data)) {                \
            if (data.timestamp == 0) {                      \
                data.timestamp = uptime;                    \
            }                                               \
            zbus_chan_pub(&chan_##TOPIC, &data, K_FOREVER); \
        }                                                   \
    }

void sim_clock_callback(const struct zbus_channel* chan)
{
    sim_clock = *(const Clock*)zbus_chan_const_msg(chan);
    // printf("sim clock callback\n");
}


extern void setup_listeners(TinyFrame * tf);

void cerebri_sim_entry_point(void) {
    printf("sim core running\n");

    static uint8_t rx1_buf[RX_BUF_SIZE];

    int serv;
    struct sockaddr_in bind_addr;
    static int counter;


    // Set up the TinyFrame library
    static TinyFrame * tf;
    tf = TF_Init(TF_MASTER);
    tf->usertag = TF_COMM_SIM_0;
    setup_listeners(tf);

    serv = socket(AF_INET, SOCK_STREAM , IPPROTO_TCP);
    if (serv < 0) {
        printf("error: socket: %d\n", errno);
        exit(1);
    }

    int status = fcntl(serv, F_SETFL, fcntl(serv, F_GETFL, 0) | O_NONBLOCK);
    if (status == -1) {
        perror("calling fcntrl");
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (bind(serv, (struct sockaddr*) &bind_addr, sizeof(bind_addr)) < 0) {
        printf("bind() failed: %d\n", errno);
        exit(1);
    }

    if (listen(serv, 5) < 0) {
        printf("error: listen: %d\n", errno);
        exit(1);
    }

    printf("listening to server on port: %d\n", BIND_PORT);

    struct timespec remaining, request;

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        client = accept(serv, (struct sockaddr*)&client_addr,
                &client_addr_len);
        fcntl(client, F_SETFL, O_NONBLOCK);


        if (client < 0) {
            printf("error: accept: %d\n", errno);
            request.tv_sec = 5;
            request.tv_nsec = 0;
            nanosleep(&request, &remaining);
            continue;
        } else {
            printf("connected\n");
        }

        inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        printf("Connection #%d from %s\n", counter++, addr_str);

        while (true) {
            int len = recv(client, rx1_buf, sizeof(rx1_buf), 0);
            printf("receiving: %d\n", len);
            TF_Accept(tf, rx1_buf, len);
            TF_Tick(tf);
            request.tv_sec = 0;
            request.tv_nsec = 1000000;
            nanosleep(&request, &remaining);
        }
    }

    /*
    while (true) {
        int64_t uptime = k_uptime_get();
        int64_t sec = uptime / 1.0e3;
        int32_t nsec = (uptime - sec * 1e3) * 1e6;

        // fast forward zephyClock time to match sim
        int64_t delta_sec = sim_clock.sim.sec - sec;
        int32_t delta_nsec = sim_clock.sim.nsec - nsec;
        int32_t wait = delta_sec * 1e6 + delta_nsec / 1e3;

        printf("sim: sec %ld nsec %d\n", sim_clock.sim.sec, sim_clock.sim.nsec);
        printf("uptime: sec %ld nsec %d\n", sec, nsec);
        printf("wait millis: %d\n", wait);
        if (wait > 0) {
            struct timespec remaining, request;
            request.tv_sec = delta_sec;
            request.tv_nsec = delta_nsec;
            nanosleep(&request, &remaining);
        }
    }
    */
}

NATIVE_TASK(cerebri_sim_entry_point, PRE_BOOT_1, 0);

// vi: ts=4 sw=4 et
