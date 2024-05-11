/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <soc.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/socket.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/utils.h>

#include <zephyr/sys/ring_buffer.h>

#include <synapse_topic_list.h>

#include "sil_context.h"

// mutex locking is not necessary, as this is single threaded
// and all consumers can only run while this process is sleeping

#define GZ_PORT 4241
#define CEREBRI_PORT 4243

RING_BUF_DECLARE(g_sil_recv, 128);
RING_BUF_DECLARE(g_sil_send, 128);

void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len);

sil_context_t g_ctx = {
    .module_name = "dream_sil_native",
    .sock = -1,
    .rx_buf = {},
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
    .external_odometry = synapse_msgs_Odometry_init_default,
    .send_actuators = synapse_msgs_Actuators_init_default,
    .send_led_array = synapse_msgs_LEDArray_init_default,
};

// public data
void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    uint32_t addr_c;
    inet_pton(AF_INET, CONFIG_NET_CONFIG_PEER_IPV4_ADDR, &addr_c);
    struct sockaddr_in client_addr = {
        .sin_addr.s_addr = addr_c,
        .sin_family = AF_INET,
        .sin_port = htons(GZ_PORT)
    };
    socklen_t client_addr_len = sizeof(client_addr);
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    if (ctx->sock >= 0 && len > 0) {
        sendto(ctx->sock, buf, len, 0,
            (struct sockaddr*)&client_addr, client_addr_len);
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
    int rc = pb_decode(&stream, synapse_msgs_SimClock_fields, &msg);
    if (rc) {
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
    int rc = pb_decode(&stream, synapse_msgs_NavSatFix_fields, &msg);
    if (rc) {
        g_ctx.nav_sat_fix = msg;
        uint8_t topic = SYNAPSE_NAV_SAT_FIX_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
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
    int rc = pb_decode(&stream, synapse_msgs_Imu_fields, &msg);
    if (rc) {
        g_ctx.imu = msg;
        uint8_t topic = SYNAPSE_IMU_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
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
    int rc = pb_decode(&stream, synapse_msgs_MagneticField_fields, &msg);
    if (rc) {
        g_ctx.magnetic_field = msg;
        uint8_t topic = SYNAPSE_MAGNETIC_FIELD_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
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
    int rc = pb_decode(&stream, synapse_msgs_BatteryState_fields, &msg);
    if (rc) {
        g_ctx.battery_state = msg;
        uint8_t topic = SYNAPSE_BATTERY_STATE_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
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
    int rc = pb_decode(&stream, synapse_msgs_WheelOdometry_fields, &msg);
    if (rc) {
        g_ctx.wheel_odometry = msg;
        uint8_t topic = SYNAPSE_WHEEL_ODOMETRY_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
    } else {
        printf("%s: wheel odometry decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    sil_context_t* ctx = (sil_context_t*)tf->userdata;
    synapse_msgs_Odometry msg = synapse_msgs_Odometry_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_Odometry_fields, &msg);
    if (rc) {
        g_ctx.external_odometry = msg;
        uint8_t topic = SYNAPSE_ODOMETRY_TOPIC;
        ring_buf_put(&g_sil_recv, &topic, 1);
    } else {
        printf("%s: external odometry decoding failed: %s\n",
            ctx->module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static void send_actuators()
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_Actuators_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int status = pb_encode(&stream, synapse_msgs_Actuators_fields, &g_ctx.send_actuators);
    if (status) {
        msg.type = SYNAPSE_ACTUATORS_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&g_ctx.tf, &msg);
    } else {
        printf("encoding failed: %s", PB_GET_ERROR(&stream));
    }
}

static void send_led_array()
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_LEDArray_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int status = pb_encode(&stream, synapse_msgs_LEDArray_fields, &g_ctx.send_led_array);
    if (status) {
        msg.type = SYNAPSE_LED_ARRAY_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&g_ctx.tf, &msg);
    } else {
        printf("encoding failed: %s", PB_GET_ERROR(&stream));
    }
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
    TF_AddTypeListener(&ctx->tf, SYNAPSE_ODOMETRY_TOPIC, odometry_listener);

    struct sockaddr_in addr;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(CEREBRI_PORT);
    ctx->sock = socket(((struct sockaddr*)&addr)->sa_family, SOCK_DGRAM, IPPROTO_UDP);

    if (ctx->sock < 0) {
        printf("failed to create UDP socket: %d\n", errno);
        exit(1);
    }

    int ret = -1;
    while (ret < 0) {
        ret = bind(ctx->sock, (struct sockaddr*)&addr, sizeof(addr));
        printf("failed to bind UDP socket: %d\n", errno);
        struct timespec request, remaining;
        request.tv_sec = 1;
        request.tv_nsec = 0;
        nanosleep(&request, &remaining);
    }
    printf("bound UDP socket\n");

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = term;
    sigaction(SIGINT, &action, NULL);

    uint32_t addr_c;
    inet_pton(AF_INET, "127.0.0.1", &addr_c);
    struct sockaddr_in client_addr = {
        .sin_addr.s_addr = addr_c,
        .sin_family = AF_INET,
        .sin_port = htons(CEREBRI_PORT)
    };
    socklen_t client_addr_len = sizeof(client_addr);

    // process incoming messages
    while (!ctx->shutdown) {

        // send data
        uint8_t topic;
        while (!ring_buf_is_empty(&g_sil_send)) {
            ring_buf_get(&g_sil_send, &topic, 1);
            if (topic == SYNAPSE_ACTUATORS_TOPIC) {
                send_actuators();
            } else if (topic == SYNAPSE_LED_ARRAY_TOPIC) {
                send_led_array();
            }
        }

        struct pollfd pollfds[] = {
            { ctx->sock, POLLIN | POLLHUP, 0 },
        };

        ret = poll(pollfds, ARRAY_SIZE(pollfds), 1000);

        if (ret == 0) {
            continue;
        } else if (ret < 0) {
            printf("poll error: %d\n", ret);
            continue;
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
            continue;
        }

        memset(ctx->rx_buf, 0, sizeof(ctx->rx_buf));

        // write received data to sim_rx_buf
        int ret = recvfrom(ctx->sock, ctx->rx_buf,
            sizeof(ctx->rx_buf), MSG_DONTWAIT,
            (struct sockaddr*)&client_addr, &client_addr_len);

        if (ret == 0) {
            continue;
        } else if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            } else {
                printf("error: %d\n", errno);
                continue;
            }
        }

        // read data
        TF_Accept(&ctx->tf, ctx->rx_buf, ret);
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
