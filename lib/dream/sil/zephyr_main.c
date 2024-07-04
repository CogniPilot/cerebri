/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0 */

// host includes
#include <pthread.h>
#include <signal.h>
#include <time.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <pb_decode.h>
#include <pb_encode.h>

#include <synapse_protobuf/sim_clock.pb.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define RX_BUF_SIZE 8192
#define TX_BUF_SIZE 8192
#define MY_STACK_SIZE 8192
#define MY_PRIORITY -10

LOG_MODULE_REGISTER(dream_sil, CONFIG_CEREBRI_DREAM_SIL_LOG_LEVEL);

RING_BUF_DECLARE(g_tx_buf, TX_BUF_SIZE);
pthread_mutex_t g_lock_tx;

RING_BUF_DECLARE(g_rx_buf, TX_BUF_SIZE);
pthread_mutex_t g_lock_rx;

struct context {
    int sock;
    pthread_t thread;
    synapse_msgs_Frame tx_frame, rx_frame;
    bool clock_initialized;
    synapse_msgs_SimClock sim_clock;
    synapse_msgs_Time clock_offset_ethernet;
    synapse_msgs_Actuators actuators;
    synapse_msgs_LEDArray led_array;
    uint64_t uptime_last;
};

extern volatile sig_atomic_t g_shutdown;

void write_sim(const uint8_t* buf, uint32_t len)
{
    pthread_mutex_lock(&g_lock_tx);
    int sent = ring_buf_put(&g_tx_buf, buf, len);
    if (sent != len) {
        LOG_ERR("failed to send: %d/%d", sent, len);
    }
    pthread_mutex_unlock(&g_lock_tx);
}

int read_sim(uint8_t* buf, uint32_t len)
{
    pthread_mutex_lock(&g_lock_rx);
    int recv = ring_buf_get(&g_rx_buf, buf, len);
    pthread_mutex_unlock(&g_lock_rx);
    return recv;
}

struct context g_ctx = {
    .sock = -1,
    .thread = 0,
    .tx_frame = synapse_msgs_Frame_init_default,
    .rx_frame = synapse_msgs_Frame_init_default,
    .clock_initialized = false,
    .sim_clock = synapse_msgs_SimClock_init_default,
    .clock_offset_ethernet = synapse_msgs_Time_init_default,
    .actuators = synapse_msgs_Actuators_init_default,
    .led_array = synapse_msgs_LEDArray_init_default,
    .uptime_last = 0
};

static K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
static struct k_thread my_thread_data;

static void send_frame(struct context* ctx, synapse_msgs_Topic topic)
{
    synapse_msgs_Frame* frame = &ctx->tx_frame;
    frame->topic = topic;
    if (topic == synapse_msgs_Topic_TOPIC_ACTUATORS) {
        frame->msg.actuators = ctx->actuators;
        frame->which_msg = synapse_msgs_Frame_actuators_tag;
    } else if (topic == synapse_msgs_Topic_TOPIC_LED_ARRAY) {
        frame->msg.led_array = ctx->led_array;
        frame->which_msg = synapse_msgs_Frame_led_array_tag;
    }
    static uint8_t tx_buf[TX_BUF_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
    if (!pb_encode_ex(&stream, synapse_msgs_Frame_fields, frame, PB_ENCODE_DELIMITED)) {
        LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
    } else {
        write_sim(tx_buf, stream.bytes_written);
    }
}

static void handle_frame(struct context* ctx)
{
    synapse_msgs_Frame* frame = &ctx->rx_frame;
    if (frame->which_msg == synapse_msgs_Frame_sim_clock_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_SIM_CLOCK) {
            struct context* ctx = &g_ctx;
            ctx->sim_clock = frame->msg.sim_clock;
            if (!ctx->clock_initialized) {
                ctx->clock_initialized = true;
                LOG_INF("sim clock received sec: %lld nsec: %d",
                    ctx->sim_clock.sim.sec, ctx->sim_clock.sim.nanosec);
                ctx->clock_offset_ethernet.sec = ctx->sim_clock.sim.sec;
                ctx->clock_offset_ethernet.nanosec = ctx->sim_clock.sim.nanosec;
                zros_topic_publish(&topic_clock_offset_ethernet, &ctx->clock_offset_ethernet);
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
            ts_board.tv_sec += ctx->clock_offset_ethernet.sec;
            ts_board.tv_nsec += ctx->clock_offset_ethernet.nanosec;

            // compute time delta from sim
            int64_t delta_sec = ctx->sim_clock.sim.sec - ts_board.tv_sec;
            int32_t delta_nsec = ctx->sim_clock.sim.nanosec - ts_board.tv_nsec;
            int64_t wait_usec = delta_sec * 1e6 + delta_nsec * 1e-3;

            LOG_DBG("\nsim: sec %lld nsec %d",
                ctx->sim_clock.sim.sec, ctx->sim_clock.sim.nanosec);
            LOG_DBG("board: sec %ld nsec %ld",
                ts_board.tv_sec, ts_board.tv_nsec);
            LOG_DBG("wait: usec %lld", wait_usec);

            // sleep to match clocks
            if (wait_usec > 0) {
                k_usleep(wait_usec);
            }
        }
    } else if (frame->which_msg == synapse_msgs_Frame_nav_sat_fix_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_NAV_SAT_FIX) {
            zros_topic_publish(&topic_nav_sat_fix, &frame->msg.nav_sat_fix);
        }
    } else if (frame->which_msg == synapse_msgs_Frame_imu_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_IMU) {
            zros_topic_publish(&topic_imu, &frame->msg.imu);
        }
    } else if (frame->which_msg == synapse_msgs_Frame_magnetic_field_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_MAGNETIC_FIELD) {
            zros_topic_publish(&topic_magnetic_field, &frame->msg.magnetic_field);
        }
    } else if (frame->which_msg == synapse_msgs_Frame_battery_state_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_BATTERY_STATE) {
            zros_topic_publish(&topic_battery_state, &frame->msg.battery_state);
        }
    } else if (frame->which_msg == synapse_msgs_Frame_wheel_odometry_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_WHEEL_ODOMETRY) {
            zros_topic_publish(&topic_wheel_odometry, &frame->msg.wheel_odometry);
        }
    } else if (frame->which_msg == synapse_msgs_Frame_odometry_tag) {
        if (frame->topic == synapse_msgs_Topic_TOPIC_ODOMETRY) {
            zros_topic_publish(&topic_odometry_ethernet, &frame->msg.odometry);
            ;
        }
    }
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

    static uint8_t buf[RX_BUF_SIZE];
    pb_istream_t stream;

    LOG_INF("running main loop");

    while (!g_shutdown) {

        if (!ctx->clock_initialized) {
            LOG_INF("waiting for sim clock");

            struct timespec request, remaining;
            request.tv_sec = 1;
            request.tv_nsec = 0;
            nanosleep(&request, &remaining);

        } else {

            // send actuators if subscription updated
            if (zros_sub_update_available(&sub_actuators)) {
                zros_sub_update(&sub_actuators);
                send_frame(ctx, synapse_msgs_Topic_TOPIC_ACTUATORS);
            }

            // send led_array if subscription updated
            if (zros_sub_update_available(&sub_led_array)) {
                zros_sub_update(&sub_led_array);
                send_frame(ctx, synapse_msgs_Topic_TOPIC_LED_ARRAY);
            }
        }

        //  receive new messages
        int len = read_sim(buf, RX_BUF_SIZE);
        if (len > 0) {
            stream = pb_istream_from_buffer(buf, len);
            while (stream.bytes_left > 0) {
                if (!pb_decode_ex(&stream, synapse_msgs_Frame_fields, &ctx->rx_frame, PB_DECODE_DELIMITED)) {
                    LOG_INF("failed to decode msg: %s\n", PB_GET_ERROR(&stream));
                    break;
                } else {
                    handle_frame(ctx);
                }
            }
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
