/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sense_mag, CONFIG_SENSE_MAG_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;
static const struct device* g_mag_dev[2];
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

    LOG_INF("mag found device %s", dev->name);
    return dev;
}

void mag_work_handler(struct k_work* work)
{
    LOG_DBG("");
    double mag_data_array[2][3] = {};
    for (int i = 0; i < 2; i++) {
        // default all data to zero
        struct sensor_value mag_value[3] = {};

        // get accel if device present
        if (g_mag_dev[i] != NULL) {
            sensor_sample_fetch(g_mag_dev[i]);
            sensor_channel_get(g_mag_dev[i], SENSOR_CHAN_MAGN_XYZ, mag_value);
            LOG_DBG("mag %d: %d.%06d %d.%06d %d.%06d", i,
                mag_value[0].val1, mag_value[0].val2,
                mag_value[1].val1, mag_value[1].val2,
                mag_value[2].val1, mag_value[2].val2);
        }

        for (int j = 0; j < 3; j++) {
            mag_data_array[i][j] = mag_value[j].val1 + mag_value[j].val2 * 1e-6;
        }
    }

    // select first mag for data for now: TODO implement voting
    double mag[3] = {
        mag_data_array[0][0],
        mag_data_array[0][1],
        mag_data_array[0][2]
    };

    // publish mag to zbus
    synapse_msgs_MagneticField msg = synapse_msgs_MagneticField_init_default;
    msg.has_header = true;
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    msg.header.seq = g_seq++;
    msg.header.has_stamp = true;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    strncpy(msg.header.frame_id, "map", sizeof(msg.header.frame_id) - 1);
    msg.has_magnetic_field = true;
    msg.magnetic_field.x = mag[0];
    msg.magnetic_field.y = mag[1];
    msg.magnetic_field.z = mag[2];
    zbus_chan_pub(&chan_out_magnetic_field, &msg, K_NO_WAIT);
}

K_WORK_DEFINE(mag_work, mag_work_handler);

void mag_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &mag_work);
}

K_TIMER_DEFINE(mag_timer, mag_timer_handler, NULL);

int sense_mag_entry_point(void)
{
    g_mag_dev[0] = sensor_check(DEVICE_DT_GET(DT_ALIAS(mag0)));
    g_mag_dev[1] = sensor_check(DEVICE_DT_GET(DT_ALIAS(mag1)));
    k_timer_start(&mag_timer, K_MSEC(20), K_MSEC(20));
    return 0;
}

K_THREAD_DEFINE(sense_mag, MY_STACK_SIZE,
    sense_mag_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
