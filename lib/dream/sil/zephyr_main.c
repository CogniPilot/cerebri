/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0 */

// host includes
#include <signal.h>
#include <time.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <synapse_protobuf/sim_clock.pb.h>
#include <synapse_tinyframe/utils.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define RX_BUF_SIZE 8192
#define TX_BUF_SIZE 8192
#define MY_STACK_SIZE 4096
#define MY_PRIORITY -10

LOG_MODULE_REGISTER(dream_sil, CONFIG_CEREBRI_DREAM_SIL_LOG_LEVEL);

RING_BUF_DECLARE(g_tx_buf, TX_BUF_SIZE);
pthread_mutex_t g_lock_tx;

RING_BUF_DECLARE(g_rx_buf, TX_BUF_SIZE);
pthread_mutex_t g_lock_rx;

struct context {
    int sock;
    pthread_t thread;
    bool clock_initialized;
    TinyFrame tf;
    synapse_msgs_SimClock sim_clock;
    synapse_msgs_Time offboard_clock_offset;
    synapse_msgs_Actuators actuators;
    synapse_msgs_LEDArray led_array;
    uint64_t uptime_last;
};

extern volatile sig_atomic_t g_shutdown;

void write_sim(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    pthread_mutex_lock(&g_lock_tx);
    int sent = ring_buf_put(&g_tx_buf, buf, len);
    if (sent != len) {
        LOG_ERR("failed to send: %d/%d", sent, len);
    }
    pthread_mutex_unlock(&g_lock_tx);
}

struct context g_ctx = {
    .sock = -1,
    .thread = 0,
    .clock_initialized = false,
    .tf = {
        .peer_bit = TF_MASTER,
        .write = write_sim,
        .userdata = &g_ctx,
    },
    .sim_clock = synapse_msgs_SimClock_init_default,
    .offboard_clock_offset = synapse_msgs_Time_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .led_array = synapse_msgs_LEDArray_init_default,
    .uptime_last = 0
};

static K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
static struct k_thread my_thread_data;

static TF_Result sim_clock_listener(TinyFrame* tf, TF_Msg* frame)
{
    struct context* ctx = (struct context*)tf->userdata;
    synapse_msgs_SimClock msg = synapse_msgs_SimClock_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_SimClock_fields, &msg);
    if (rc) {
        ctx->sim_clock = msg;
        if (!ctx->clock_initialized) {
            ctx->clock_initialized = true;
            LOG_INF("sim clock received sec: %lld nsec: %d",
                msg.sim.sec, msg.sim.nanosec);
            ctx->offboard_clock_offset.sec = msg.sim.sec;
            ctx->offboard_clock_offset.nanosec = msg.sim.nanosec;
        }

        // compute board time
        uint64_t uptime = k_uptime_get();
        int uptime_delta = uptime - ctx->uptime_last;
        if (uptime_delta != 4 && uptime_delta != 0) {
            LOG_WRN("uptime delta: %d\n", uptime_delta);
        }
        ctx->uptime_last = uptime;
        struct timespec ts_board;
        ts_board.tv_sec = uptime / 1.0e3;
        ts_board.tv_nsec = (uptime - ts_board.tv_sec * 1e3) * 1e6;
        ts_board.tv_sec += ctx->offboard_clock_offset.sec;
        ts_board.tv_nsec += ctx->offboard_clock_offset.nanosec;

        // compute time delta from sim
        int64_t delta_sec = ctx->sim_clock.sim.sec - ts_board.tv_sec;
        int32_t delta_nsec = ctx->sim_clock.sim.nanosec - ts_board.tv_nsec;
        int64_t wait_usec = delta_sec * 1e6 + delta_nsec * 1e-3;

        // sleep to match clocks
        if (wait_usec > 0) {
            LOG_DBG("sim: sec %lld nsec %d\n",
                ctx->sim_clock.sim.sec, ctx->sim_clock.sim.nanosec);
            LOG_DBG("board: sec %ld nsec %ld\n",
                ts_board.tv_sec, ts_board.tv_nsec);
            LOG_DBG("wait: usec %lld\n", wait_usec);
            k_usleep(wait_usec);
        }

    } else {
        LOG_ERR("sim_clock decoding failed: %s", PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result nav_sat_fix_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_NavSatFix_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_nav_sat_fix, &msg);
    } else {
        LOG_ERR("navsat decoding failed: %s", PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result imu_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_Imu msg = synapse_msgs_Imu_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_Imu_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_imu, &msg);
    } else {
        LOG_ERR("imu decoding failed: %s", PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result magnetic_field_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_MagneticField msg = synapse_msgs_MagneticField_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_MagneticField_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_magnetic_field, &msg);
    } else {
        LOG_ERR("magnetic field decoding failed: %s",
            PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result battery_state_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_BatteryState msg = synapse_msgs_BatteryState_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_BatteryState_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_battery_state, &msg);
    } else {
        LOG_ERR("battery state decoding failed: %s",
            PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result wheel_odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_WheelOdometry msg = synapse_msgs_WheelOdometry_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_WheelOdometry_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_wheel_odometry, &msg);
    } else {
        LOG_ERR("wheel odometry decoding failed: %s\n",
            PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static TF_Result odometry_listener(TinyFrame* tf, TF_Msg* frame)
{
    synapse_msgs_Odometry msg = synapse_msgs_Odometry_init_default;
    pb_istream_t stream = pb_istream_from_buffer(frame->data, frame->len);
    int rc = pb_decode(&stream, synapse_msgs_Odometry_fields, &msg);
    if (rc) {
        zros_topic_publish(&topic_offboard_odometry, &msg);
    } else {
        LOG_ERR("external odometry decoding failed: %s",
            PB_GET_ERROR(&stream));
    }
    return TF_STAY;
}

static void send_actuators(struct context* ctx)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_Actuators_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int status = pb_encode(&stream, synapse_msgs_Actuators_fields, &ctx->actuators);
    if (status) {
        msg.type = SYNAPSE_ACTUATORS_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&ctx->tf, &msg);
    } else {
        LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
    }
}

static void send_led_array(struct context* ctx)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_LEDArray_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int status = pb_encode(&stream, synapse_msgs_LEDArray_fields, &ctx->led_array);
    if (status) {
        msg.type = SYNAPSE_LED_ARRAY_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&ctx->tf, &msg);
    } else {
        LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
    }
}

TF_Result generic_listener(TinyFrame* tf, TF_Msg* frame)
{
    return TF_STAY;
}

static void zephyr_sim_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");

    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    struct zros_node node;
    struct zros_sub sub_actuators, sub_led_array;

    zros_node_init(&node, "dream_sil");
    zros_sub_init(&sub_actuators, &node, &topic_actuators, &ctx->actuators, 10);
    zros_sub_init(&sub_led_array, &node, &topic_led_array, &ctx->led_array, 10);

    // setup tinyframe
    int ret = TF_InitStatic(&ctx->tf, TF_MASTER, write_sim);
    if (ret < 0) {
        LOG_ERR("tf init failed: %d", ret);
        return;
    }
    TF_AddGenericListener(&ctx->tf, generic_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_SIM_CLOCK_TOPIC, sim_clock_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_NAV_SAT_FIX_TOPIC, nav_sat_fix_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_IMU_TOPIC, imu_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_MAGNETIC_FIELD_TOPIC, magnetic_field_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_BATTERY_STATE_TOPIC, battery_state_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_WHEEL_ODOMETRY_TOPIC, wheel_odometry_listener);
    TF_AddTypeListener(&ctx->tf, SYNAPSE_ODOMETRY_TOPIC, odometry_listener);

    static uint8_t buf[RX_BUF_SIZE];

    while (!g_shutdown) {
        LOG_INF("waiting for sim clock");

        struct timespec request, remaining;
        request.tv_sec = 1;
        request.tv_nsec = 0;
        nanosleep(&request, &remaining);

        //  publish new messages
        memset(buf, 0, RX_BUF_SIZE);
        pthread_mutex_lock(&g_lock_rx);
        int len = ring_buf_get(&g_rx_buf, buf, RX_BUF_SIZE);
        pthread_mutex_unlock(&g_lock_rx);
        if (len > 0) {
            // LOG_INF("zephyr received %d", len);
            TF_Accept(&ctx->tf, buf, len);
        }

        if (ctx->clock_initialized) {
            LOG_DBG("sim clock initialized");
            zros_topic_publish(&topic_offboard_clock_offset, &ctx->offboard_clock_offset);
            break;
        } else {
            struct timespec request, remaining;
            request.tv_sec = 1;
            request.tv_nsec = 0;
            nanosleep(&request, &remaining);
        }
    }

    LOG_INF("running main loop");

    while (!g_shutdown) {

        // send actuators if subscription updated
        if (zros_sub_update_available(&sub_actuators)) {
            zros_sub_update(&sub_actuators);
            send_actuators(ctx);
        }

        // send led_array if subscription updated
        if (zros_sub_update_available(&sub_led_array)) {
            zros_sub_update(&sub_led_array);
            send_led_array(ctx);
        }

        //  publish new messages
        int len = ring_buf_get(&g_rx_buf, buf, RX_BUF_SIZE);
        if (len > 0) {
            // LOG_INF("zephyr received %d\n", len);
            TF_Accept(&ctx->tf, buf, len);
        } else {
            // wait for new meessage
            struct timespec request, remaining;
            request.tv_sec = 0;
            request.tv_nsec = 1000000;
            nanosleep(&request, &remaining);
        }
    }
    LOG_INF("finished\n");

    zros_sub_fini(&sub_actuators);
    zros_sub_fini(&sub_led_array);
    zros_node_fini(&node);
}

static int start()
{
    k_tid_t tid = k_thread_create(&my_thread_data, my_stack_area,
        K_THREAD_STACK_SIZEOF(my_stack_area),
        zephyr_sim_entry_point,
        &g_ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "dream_sil");
    k_thread_start(tid);
    return 0;
}

SYS_INIT(start, APPLICATION, 0);

// vi: ts=4 sw=4 et
