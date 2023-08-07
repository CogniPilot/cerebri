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

#include <synapse_protobuf/altimeter.pb.h>
#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/sim_clock.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

#include <zephyr/sys/ring_buffer.h>

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

RING_BUF_DECLARE(g_msg_updates, 120);

// mutex locking is not necessary, as this is single threaded
// and all consumers can only run while this process is sleeping
synapse_msgs_SimClock g_sim_clock = synapse_msgs_SimClock_init_default;
bool g_clock_initialized = false;
synapse_msgs_Time g_clock_offset = synapse_msgs_Time_init_default;
TinyFrame g_tf = {
    .peer_bit = TF_MASTER,
    .write = write_sim,
    .userdata = &g_priv.client,
};
synapse_msgs_NavSatFix g_in_nav_sat_fix = synapse_msgs_NavSatFix_init_default;
synapse_msgs_Imu g_in_imu = synapse_msgs_Imu_init_default;
synapse_msgs_MagneticField g_in_magnetic_field = synapse_msgs_MagneticField_init_default;
synapse_msgs_BatteryState g_in_battery_state = synapse_msgs_BatteryState_init_default;
synapse_msgs_Altimeter g_in_altimeter = synapse_msgs_Altimeter_init_default;
synapse_msgs_WheelOdometry g_in_wheel_odometry = synapse_msgs_WheelOdometry_init_default;

static TF_Result sim_clock_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_SimClock msg = synapse_msgs_SimClock_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_SimClock_fields, &msg);
    if (status) {
        g_sim_clock = msg;
        if (!g_clock_initialized) {
            g_clock_initialized = true;
            printf("%s: sim clock received sec: %lld nsec: %d\n",
                g_priv.module_name, msg.sim.sec, msg.sim.nanosec);
            g_clock_offset.sec = msg.sim.sec;
            g_clock_offset.nanosec = msg.sim.nanosec;
        }
    } else {
        printf("%s: sim_clock decoding failed: %s\n", g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_NavSatFix_fields, &msg);
    if (status) {
        g_in_nav_sat_fix = msg;
        uint8_t topic = SYNAPSE_IN_NAV_SAT_FIX_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: navsat decoding failed: %s\n",
            g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result imu_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_Imu msg = synapse_msgs_Imu_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_Imu_fields, &msg);
    if (status) {
        g_in_imu = msg;
        uint8_t topic = SYNAPSE_IN_IMU_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: imu decoding failed: %s\n",
            g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result magnetic_field_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_MagneticField msg = synapse_msgs_MagneticField_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_MagneticField_fields, &msg);
    if (status) {
        g_in_magnetic_field = msg;
        uint8_t topic = SYNAPSE_IN_MAGNETIC_FIELD_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: magnetic field decoding failed: %s\n",
            g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result battery_state_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_BatteryState msg = synapse_msgs_BatteryState_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_BatteryState_fields, &msg);
    if (status) {
        g_in_battery_state = msg;
        uint8_t topic = SYNAPSE_IN_BATTERY_STATE_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: battery state decoding failed: %s\n",
            g_priv.module_name, PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result wheel_odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_WheelOdometry msg = synapse_msgs_WheelOdometry_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int status = pb_decode(&stream, synapse_msgs_WheelOdometry_fields, &msg);
    if (status) {
        g_in_wheel_odometry = msg;
        uint8_t topic = SYNAPSE_IN_WHEEL_ODOMETRY_TOPIC;
        ring_buf_put(&g_msg_updates, &topic, 1);
    } else {
        printf("%s: wheel odometry decoding failed: %s\n",
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
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_SIM_CLOCK_TOPIC, sim_clock_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_NAV_SAT_FIX_TOPIC, nav_sat_fix_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_IMU_TOPIC, imu_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_MAGNETIC_FIELD_TOPIC, magnetic_field_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_BATTERY_STATE_TOPIC, battery_state_listener);
    TF_AddTypeListener(&g_tf, SYNAPSE_IN_WHEEL_ODOMETRY_TOPIC, wheel_odometry_listener);

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
