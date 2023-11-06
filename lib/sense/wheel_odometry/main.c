/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>
#include <cerebri/synapse/zbus/channels.h>

LOG_MODULE_REGISTER(sense_wheel_odometry, CONFIG_CEREBRI_SENSE_WHEEL_ODOMETRY_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;
static const struct device* g_dev[1];
static int32_t g_seq = 0;
static int g_n_sensors = 1;

void wheel_odometry_work_handler(struct k_work* work)
{
    LOG_DBG("");
    double data_array[1] = {};
    for (int i = 0; i < g_n_sensors; i++) {
        // default all data to zero
        struct sensor_value value = {};

        // get accel if device present
        if (g_dev[i] != NULL) {
            sensor_sample_fetch(g_dev[i]);
            sensor_channel_get(g_dev[i], SENSOR_CHAN_ROTATION, &value);
            LOG_DBG("rotation %d: %d.%06d", i, value.val1, value.val2);
        }

        for (int j = 0; j < g_n_sensors; j++) {
            data_array[i] = value.val1 + value.val2 * 1e-6;
        }
    }

    // select first wheel encoder for data for now: TODO implement voting
    double rotation = data_array[0];

    // publish mag to zbus
    synapse_msgs_WheelOdometry msg = synapse_msgs_WheelOdometry_init_default;

    msg.has_header = true;
    stamp_header(&msg.header, k_uptime_ticks());
    msg.header.seq = g_seq++;
    msg.header.has_stamp = true;
    strncpy(msg.header.frame_id, "base_link", sizeof(msg.header.frame_id) - 1);

    msg.rotation = -rotation; // account for negative rotation of encoder
    zbus_chan_pub(&chan_wheel_odometry, &msg, K_NO_WAIT);
}

K_WORK_DEFINE(wheel_odometry_work, wheel_odometry_work_handler);

void wheel_odometry_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &wheel_odometry_work);
}

K_TIMER_DEFINE(wheel_odometry_timer, wheel_odometry_timer_handler, NULL);

int sense_wheel_odometry_entry_point(void)
{
    g_dev[0] = get_device(DEVICE_DT_GET(DT_ALIAS(wheel_odometry0)));
    k_timer_start(&wheel_odometry_timer, K_MSEC(10), K_MSEC(10));
    return 0;
}

K_THREAD_DEFINE(sense_wheel_odometry, MY_STACK_SIZE,
    sense_wheel_odometry_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
