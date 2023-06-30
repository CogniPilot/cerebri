/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/socket.h>

#include <pb_decode.h>
#include <synapse_protobuf/clock.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

#define BIND_PORT 4241
#define RX_BUF_SIZE 1024

// private data
struct private_module_context {
    const char* module_name;
    int serv;
    int client;
    pthread_t thread;
};

static struct private_module_context g_priv = {
    .module_name = "dream_sitl_native",
    .serv = 0,
    .client = 0,
};

// public data
void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    int client = *(int*)(tf->userdata);
    if (len > 0) {
        send(client, buf, len, 0);
    }
}

// mutex locking is not necessary, as this is single threaded
// and all consumers can only run while this process is sleeping
synapse_msgs_Clock g_sim_clock = synapse_msgs_Clock_init_default;
bool g_clock_initialized = false;
synapse_msgs_Timestamp g_clock_offset = synapse_msgs_Timestamp_init_default;
TinyFrame g_tf = {
    .peer_bit = TF_MASTER,
    .write = write_sim,
    .userdata = &g_priv.client,
};
synapse_msgs_NavSatFix g_in_nav_sat_fix = synapse_msgs_NavSatFix_init_default;

static TF_Result sim_clock_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_Clock msg = synapse_msgs_Clock_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_Clock_fields, &msg);
    if (status) {
        g_sim_clock = msg;
        if (!g_clock_initialized) {
            g_clock_initialized = true;
            printf("%s: sim clock received sec: %lld nsec: %d\n",
                g_priv.module_name, msg.sim.sec, msg.sim.nsec);
            g_clock_offset.seconds = msg.sim.sec;
            g_clock_offset.nanos = msg.sim.nsec;
        }
    } else {
        printf("dream_sitl: sim_clock decoding failed: %s\n", PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result sim_nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_NavSatFix_fields, &msg);
    if (status) {
        g_in_nav_sat_fix = msg;
    } else {
        printf("%s: sim_clock decoding failed: %s\n",
            g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

TF_Result generic_listener(TinyFrame* tf, TF_Msg* frame)
{
    return TF_STAY;
}

void* native_sim_entry_point(void* data)
{
    printf("%s: sim core running\n", g_priv.module_name);

    // setup tinyframe
    TF_AddGenericListener(&g_tf, generic_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_SIM_CLOCK_TOPIC, sim_clock_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_NAVSAT_TOPIC, sim_nav_sat_fix_listener);
    struct sockaddr_in bind_addr;
    static int counter;

    g_priv.serv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (g_priv.serv < 0) {
        printf("%s: error: socket: %d\n", g_priv.module_name, errno);
        exit(1);
    }

    int status = fcntl(g_priv.serv, F_SETFL, fcntl(g_priv.serv, F_GETFL, 0) | O_NONBLOCK);
    if (status == -1) {
        perror("calling fcntrl");
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (bind(g_priv.serv, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("%s: bind() failed: %d\n", g_priv.module_name, errno);
        exit(1);
    }

    if (listen(g_priv.serv, 5) < 0) {
        printf("%s: error: listen: %d\n", g_priv.module_name, errno);
        exit(1);
    }

    printf("%s: listening to server on port: %d\n", g_priv.module_name, BIND_PORT);

    struct timespec remaining, request;

    printf("%s: waiting for client connection\n", g_priv.module_name);
    while (true) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        g_priv.client = accept(g_priv.serv, (struct sockaddr*)&client_addr,
            &client_addr_len);
        fcntl(g_priv.client, F_SETFL, O_NONBLOCK);

        if (g_priv.client < 0) {
            request.tv_sec = 1;
            request.tv_nsec = 0;
            nanosleep(&request, &remaining);
            continue;
        }

        inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        printf("%s: connection #%d from %s\n", g_priv.module_name, counter++, addr_str);

        // process incoming messages
        while (true) {
            // write received data to sim_rx_buf
            uint8_t data[RX_BUF_SIZE];
            int len = recv(g_priv.client, data, RX_BUF_SIZE, 0);
            if (len > 0) {
                TF_Accept(&g_tf, data, len);
            }
            request.tv_sec = 0;
            request.tv_nsec = 1000000; // 1 ms
            nanosleep(&request, &remaining);
        }
    }
}

void native_sim_start_task(void)
{
    pthread_create(&g_priv.thread, NULL, native_sim_entry_point, NULL);
}

void native_sim_stop_task(void)
{
    pthread_join(g_priv.thread, NULL);
}

// native tasks
NATIVE_TASK(native_sim_start_task, PRE_BOOT_1, 0);
NATIVE_TASK(native_sim_stop_task, ON_EXIT, 0);

// vi: ts=4 sw=4 et
