/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/socket.h>

#include <pb_decode.h>
#include <synapse_tinyframe/SynapseTopics.h>
#include <zephyr/sys/ring_buffer.h>

#include "sil_context.h"

// mutex locking is not necessary, as this is single threaded
// and all consumers can only run while this process is sleeping

#define BIND_PORT 4241
#define RX_BUF_SIZE 2048

RING_BUF_DECLARE(g_msg_updates, 1024);

void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len);

sil_context_t g_ctx = {
    .module_name = "dream_sil_native",
    .serv = 0,
    .client = 0,
    .thread = 0,
    .clock_initialized = false,
    .tf = {
        .peer_bit = TF_MASTER,
        .write = write_sim,
        .userdata = &g_ctx,
    },
    .sim_clock = synapse_msgs_SimClock_init_default,
    .nav_sat_fix = synapse_msgs_NavSatFix_init_default,
    .imu = synapse_msgs_Imu_init_default,
    .magnetic_field = synapse_msgs_MagneticField_init_default,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .altimeter = synapse_msgs_Altimeter_init_default,
    .wheel_odometry = synapse_msgs_WheelOdometry_init_default,
};

// public data
void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    int client = ctx->client;
    if (len > 0) {
        send(client, buf, len, 0);
    }
}

static void term(int signum)
{
    g_ctx.shutdown = 1;
    printf("handling term\n");
}

static TF_Result sim_clock_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_SimClock msg = synapse_msgs_SimClock_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_SimClock_fields, &msg);
    if (status) {
        g_ctx.sim_clock = msg;
        if (!g_ctx.clock_initialized) {
            g_ctx.clock_initialized = true;
            printf("%s: sim clock received sec: %lld nsec: %d\n",
                ctx->module_name, msg.sim.sec, msg.sim.nanosec);
            g_ctx.clock_offset.sec = msg.sim.sec;
            g_ctx.clock_offset.nanosec = msg.sim.nanosec;
        }
    } else {
        printf("%s: sim_clock decoding failed: %s\n", ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_NavSatFix_fields, &msg);
    if (status) {
        g_ctx.nav_sat_fix = msg;
        uint8_t topic = SYNAPSE_NAV_SAT_FIX_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: navsat decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result imu_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_Imu msg = synapse_msgs_Imu_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_Imu_fields, &msg);
    if (status) {
        g_ctx.imu = msg;
        uint8_t topic = SYNAPSE_IMU_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: imu decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result magnetic_field_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_MagneticField msg = synapse_msgs_MagneticField_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_MagneticField_fields, &msg);
    if (status) {
        g_ctx.magnetic_field = msg;
        uint8_t topic = SYNAPSE_MAGNETIC_FIELD_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: magnetic field decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result battery_state_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_BatteryState msg = synapse_msgs_BatteryState_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_BatteryState_fields, &msg);
    if (status) {
        g_ctx.battery_state = msg;
        uint8_t topic = SYNAPSE_BATTERY_STATE_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: battery state decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result wheel_odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_WheelOdometry msg = synapse_msgs_WheelOdometry_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_WheelOdometry_fields, &msg);
    if (status) {
        g_ctx.wheel_odometry = msg;
        uint8_t topic = SYNAPSE_WHEEL_ODOMETRY_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: wheel odometry decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

TF_Result generic_listener(TinyFrame* tf, TF_Msg* frame)
{
    return TF_STAY;
}

void* native_sim_entry_point(void* p0)
{
    sil_context_t* ctx = p0;
    printf("%s: sim core running\n", ctx->module_name);

    // setup tinyframe
    TF_AddGenericListener(&ctx->tf, generic_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_SIM_CLOCK_TOPIC, sim_clock_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_NAV_SAT_FIX_TOPIC, nav_sat_fix_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_IMU_TOPIC, imu_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_MAGNETIC_FIELD_TOPIC, magnetic_field_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_BATTERY_STATE_TOPIC, battery_state_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_WHEEL_ODOMETRY_TOPIC, wheel_odometry_listener);

    struct sockaddr_in bind_addr;
    static int counter;

    ctx->serv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    const int trueFlag = 1;
    if (setsockopt(ctx->serv, SOL_SOCKET, SO_REUSEADDR, &trueFlag, sizeof(int)) < 0) {
        printf("failed to set socket options\n");
        exit(1);
    };

    if (ctx->serv < 0) {
        printf("%s: error: socket: %d\n", ctx->module_name, errno);
        exit(1);
    }

    int status = fcntl(ctx->serv, F_SETFL, fcntl(ctx->serv, F_GETFL, 0) | O_NONBLOCK);
    if (status == -1) {
        perror("calling fcntrl");
        exit(1);
    }

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(BIND_PORT);

    if (bind(ctx->serv, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("%s: bind() failed: %d\n", ctx->module_name, errno);
        exit(1);
    }

    if (listen(ctx->serv, 5) < 0) {
        printf("%s: error: listen: %d\n", ctx->module_name, errno);
        exit(1);
    }

    printf("%s: listening to server on port: %d\n", ctx->module_name, BIND_PORT);

    struct timespec remaining, request;

    printf("%s: waiting for client connection\n", ctx->module_name);

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = term;
    sigaction(SIGINT, &action, NULL);

    while (!ctx->shutdown) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char addr_str[32];
        ctx->client = accept(ctx->serv, (struct sockaddr*)&client_addr,
            &client_addr_len);
        fcntl(ctx->client, F_SETFL, O_NONBLOCK);

        if (ctx->client < 0) {
            request.tv_sec = 1;
            request.tv_nsec = 0;
            nanosleep(&request, &remaining);
            continue;
        }

        inet_ntop(client_addr.sin_family, &client_addr.sin_addr,
            addr_str, sizeof(addr_str));
        printf("%s: connection #%d from %s\n", ctx->module_name, counter++, addr_str);

        // process incoming messages
        while (!ctx->shutdown) {
            // write received data to sim_rx_buf
            uint8_t data[RX_BUF_SIZE];
            int len = recv(ctx->client, data, RX_BUF_SIZE, 0);
            if (len > 0) {
                TF_Accept(&ctx->tf, data, len);
            }
            request.tv_sec = 0;
            request.tv_nsec = 1000000; // 1 ms
            nanosleep(&request, &remaining);
        }
    }

    printf("native main exitting\n");
    exit(0);
    return 0;
}

void native_sim_start_task(void)
{
    printf("native sim start task\n");
    pthread_create(&g_ctx.thread, NULL, native_sim_entry_point, &g_ctx);
}

void native_sim_stop_task(void)
{
    printf("native sim stop task\n");
    pthread_join(g_ctx.thread, NULL);
}

// native tasks
NATIVE_TASK(native_sim_start_task, PRE_BOOT_1, 0);
NATIVE_TASK(native_sim_stop_task, ON_EXIT_PRE, 1);

// vi: ts=4 sw=4 et
