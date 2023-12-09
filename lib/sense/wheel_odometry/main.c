/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_protobuf/wheel_odometry.pb.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(sense_wheel_odometry, CONFIG_CEREBRI_SENSE_WHEEL_ODOMETRY_LOG_LEVEL);

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;

#define N_SENSORS 1

void wheel_odometry_work_handler(struct k_work* work);

typedef struct context {
    struct k_work work_item;
    const struct device* device[N_SENSORS];
    struct zros_node node;
    struct zros_pub pub;
    synapse_msgs_WheelOdometry data;
} context_t;

static context_t g_ctx = {
    .work_item = Z_WORK_INITIALIZER(wheel_odometry_work_handler),
    .device = {},
    .node = {},
    .pub = {},
    .data = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default,
        },
        .rotation = 0,
    }
};

void wheel_odometry_work_handler(struct k_work* work)
{
    context_t* ctx = CONTAINER_OF(work, context_t, work_item);
    double data_array[N_SENSORS];
    for (int i = 0; i < N_SENSORS; i++) {
        // default all data to zero
        struct sensor_value value = {};

        // get accel if device present
        if (ctx->device[i] != NULL) {
            sensor_sample_fetch(ctx->device[i]);
            sensor_channel_get(ctx->device[i], SENSOR_CHAN_ROTATION, &value);
            LOG_DBG("rotation %d: %d.%06d", i, value.val1, value.val2);
        }

        for (int j = 0; j < N_SENSORS; j++) {
            data_array[i] = value.val1 + value.val2 * 1e-6;
        }
    }

    // select first wheel encoder for data for now: TODO implement voting
    double rotation = -data_array[0]; // account for negative rotation of encoder

    // publish msg
    stamp_header(&ctx->data.header, k_uptime_ticks());
    ctx->data.header.seq++;
    ctx->data.rotation = rotation;
    zros_pub_update(&ctx->pub);
}

void wheel_odometry_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &g_ctx.work_item);
}

K_TIMER_DEFINE(wheel_odometry_timer, wheel_odometry_timer_handler, NULL);

int sense_wheel_odometry_entry_point(context_t* ctx)
{
    LOG_INF("init");
    ctx->device[0] = get_device(DEVICE_DT_GET(DT_ALIAS(wheel_odometry0)));
    zros_node_init(&ctx->node, "sense_wheel_odometry");
    zros_pub_init(&ctx->pub, &ctx->node, &topic_wheel_odometry, &ctx->data);
    k_timer_start(&wheel_odometry_timer, K_MSEC(10), K_MSEC(10));
    return 0;
}

K_THREAD_DEFINE(sense_wheel_odometry, MY_STACK_SIZE,
    sense_wheel_odometry_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 100);

// vi: ts=4 sw=4 et
