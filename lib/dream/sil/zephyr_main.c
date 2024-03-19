/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>

#include <pb_encode.h>

#include <synapse_protobuf/sim_clock.pb.h>
#include <synapse_tinyframe/utils.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include "sil_context.h"

LOG_MODULE_REGISTER(dream_sil, CONFIG_CEREBRI_DREAM_SIL_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY -10

extern sil_context_t g_ctx;
extern struct ring_buf g_msg_updates;
static K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
static struct k_thread my_thread_data;

static void zephyr_sim_entry_point(void* p0, void* p1, void* p2)
{
    struct zros_node node;
    struct zros_sub sub_actuators, sub_led_array;
    synapse_msgs_Actuators actuators;
    synapse_msgs_LEDArray led_array;

    zros_node_init(&node, "dream_sil");
    zros_sub_init(&sub_actuators, &node, &topic_actuators, &actuators, 10);
    zros_sub_init(&sub_led_array, &node, &topic_led_array, &led_array, 10);

    sil_context_t* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    LOG_INF("zephyr sim entry point");
    LOG_INF("waiting for sim clock");
    while (!ctx->shutdown) {
        // if clock not initialized, wait 1 second
        synapse_msgs_SimClock sim_clock;
        struct timespec request, remaining;
        request.tv_sec = 1;
        request.tv_nsec = 0;
        nanosleep(&request, &remaining);
        sim_clock = ctx->sim_clock;
        if (ctx->clock_initialized) {
            LOG_DBG("sim clock initialized");
            zros_topic_publish(&topic_clock_offset, &ctx->clock_offset);
            break;
        }
    }

    LOG_DBG("running main loop");
    while (!ctx->shutdown) {

        //  publish new messages
        uint8_t topic;
        while (!ring_buf_is_empty(&g_msg_updates)) {
            ring_buf_get(&g_msg_updates, &topic, 1);
            if (topic == SYNAPSE_NAV_SAT_FIX_TOPIC) {
                zros_topic_publish(&topic_nav_sat_fix, &ctx->nav_sat_fix);
            } else if (topic == SYNAPSE_MAGNETIC_FIELD_TOPIC) {
                zros_topic_publish(&topic_magnetic_field, &ctx->magnetic_field);
            } else if (topic == SYNAPSE_IMU_TOPIC) {
                zros_topic_publish(&topic_imu, &ctx->imu);
            } else if (topic == SYNAPSE_ALTIMETER_TOPIC) {
                zros_topic_publish(&topic_altimeter, &ctx->altimeter);
            } else if (topic == SYNAPSE_BATTERY_STATE_TOPIC) {
                zros_topic_publish(&topic_battery_state, &ctx->battery_state);
            } else if (topic == SYNAPSE_WHEEL_ODOMETRY_TOPIC) {
                zros_topic_publish(&topic_wheel_odometry, &ctx->wheel_odometry);
            } else if (topic == SYNAPSE_ODOMETRY_TOPIC) {
                zros_topic_publish(&topic_external_odometry, &ctx->external_odometry);
            }
        }

        // send actuators if subscription updated
        if (zros_sub_update_available(&sub_actuators)) {
            zros_sub_update(&sub_actuators);
            TF_Msg msg;
            TF_ClearMsg(&msg);
            uint8_t buf[synapse_msgs_Actuators_size];
            pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
            int status = pb_encode(&stream, synapse_msgs_Actuators_fields, &actuators);
            if (status) {
                msg.type = SYNAPSE_ACTUATORS_TOPIC;
                msg.data = buf;
                msg.len = stream.bytes_written;
                TF_Send(&ctx->tf, &msg);
            } else {
                LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
            }
        }

        // send led array if subscription updated
        if (zros_sub_update_available(&sub_led_array)) {
            zros_sub_update(&sub_led_array);
            TF_Msg msg;
            TF_ClearMsg(&msg);
            uint8_t buf[synapse_msgs_LEDArray_size];
            pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
            int status = pb_encode(&stream, synapse_msgs_LEDArray_fields, &led_array);
            if (status) {
                msg.type = SYNAPSE_LED_ARRAY_TOPIC;
                msg.data = buf;
                msg.len = stream.bytes_written;
                TF_Send(&ctx->tf, &msg);
            } else {
                LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
            }
        }

        // compute board time
        uint64_t uptime = k_uptime_get();
        struct timespec ts_board;
        ts_board.tv_sec = uptime / 1.0e3;
        ts_board.tv_nsec = (uptime - ts_board.tv_sec * 1e3) * 1e6;
        ts_board.tv_sec += ctx->clock_offset.sec;
        ts_board.tv_nsec += ctx->clock_offset.nanosec;

        // compute time delta from sim
        int64_t delta_sec = ctx->sim_clock.sim.sec - ts_board.tv_sec;
        int32_t delta_nsec = ctx->sim_clock.sim.nanosec - ts_board.tv_nsec;
        int64_t wait_msec = delta_sec * 1e3 + delta_nsec * 1e-6;

        // sleep to match clocks
        if (wait_msec > 0) {
            LOG_DBG("sim: sec %lld nsec %d\n",
                ctx->sim_clock.sim.sec, ctx->sim_clock.sim.nanosec);
            LOG_DBG("board: sec %ld nsec %ld\n",
                ts_board.tv_sec, ts_board.tv_nsec);
            LOG_DBG("wait: msec %lld\n", wait_msec);
            k_msleep(wait_msec);
        } else {
            struct timespec request, remaining;
            request.tv_sec = 0;
            request.tv_nsec = 1000000;
            nanosleep(&request, &remaining);
        }
    }
    printf("zephyr main loop finished\n");
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

SYS_INIT(start, POST_KERNEL, 0);

// vi: ts=4 sw=4 et
