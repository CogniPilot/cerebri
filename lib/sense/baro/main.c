/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <math.h>

LOG_MODULE_REGISTER(sense_baro, CONFIG_SENSE_BARO_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 6

extern struct k_work_q g_low_priority_work_q;
static const struct device* g_baro_dev[2];
static int32_t g_seq = 0;

static const struct device* sensor_check(const struct device* const dev)
{
    if (dev == NULL) {
        /* No such node, or the node does not have status "okay". */
        LOG_ERR("no device found");
        return NULL;
    }

    if (!device_is_ready(dev)) {
        LOG_ERR("device %s is not ready, check the driver initialization logs for errors",
            dev->name);
        return NULL;
    }

    LOG_INF("baro found device %s", dev->name);
    return dev;
}

void baro_work_handler(struct k_work* work)
{
    double baro_data_array[2][2] = {};
    for (int i = 0; i < 2; i++) {
        // default all data to zero
        struct sensor_value baro_press = {};
        struct sensor_value baro_temp = {};

        // get accel if device present
        if (g_baro_dev[i] != NULL) {
            sensor_sample_fetch(g_baro_dev[i]);
            sensor_channel_get(g_baro_dev[i], SENSOR_CHAN_PRESS, &baro_press);
            sensor_channel_get(g_baro_dev[i], SENSOR_CHAN_AMBIENT_TEMP, &baro_temp);
            LOG_DBG("baro %d: %d.%06d %d.%06d", i,
                baro_press.val1, baro_press.val2,
                baro_temp.val1, baro_temp.val2);
        }

        baro_data_array[i][0] = baro_press.val1 + baro_press.val2 * 1e-6;
        baro_data_array[i][1] = baro_temp.val1 + baro_temp.val2 * 1e-6;
    }

    // select first baro for data for now: TODO implement voting
    // TODO add barmetric formula equation
    double press = baro_data_array[0][0];
    double temp = 15.0; // standard atmosphere temp in C
    const float sea_press = 101.325;
    double alt = ((pow((sea_press / press), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
    // LOG_DBG("press %10.4f, temp: %10.4f, alt: %10.4f", press, temp, alt);

    // publish mag to zbus
    synapse_msgs_Altimeter msg = synapse_msgs_Altimeter_init_default;
    msg.has_header = true;
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    msg.header.seq = g_seq++;
    msg.header.has_stamp = true;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    strncpy(msg.header.frame_id, "map", sizeof(msg.header.frame_id) - 1);
    msg.vertical_position = alt;
    msg.vertical_velocity = 0;
    msg.vertical_reference = 0;
    zbus_chan_pub(&chan_out_altimeter, &msg, K_NO_WAIT);
}

K_WORK_DEFINE(baro_work, baro_work_handler);

void baro_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_low_priority_work_q, &baro_work);
}

K_TIMER_DEFINE(baro_timer, baro_timer_handler, NULL);

int sense_baro_entry_point(void)
{
    g_baro_dev[0] = sensor_check(DEVICE_DT_GET(DT_ALIAS(baro0)));
    g_baro_dev[1] = sensor_check(DEVICE_DT_GET(DT_ALIAS(baro1)));
    k_timer_start(&baro_timer, K_MSEC(1000), K_MSEC(1000));
    return 0;
}

K_THREAD_DEFINE(sense_baro, MY_STACK_SIZE,
    sense_baro_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
