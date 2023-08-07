/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sense_wheel_odometry, CONFIG_SENSE_WHEEL_ODOMETRY_LOG_LEVEL);

#define MY_STACK_SIZE 520
#define MY_PRIORITY 6

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

    LOG_INF("wheel odometry found device %s", dev->name);
    return dev;
}

int sense_wheel_odometry_entry_point(void)
{
    const struct device* const dev[] = {
        sensor_check(DEVICE_DT_GET(DT_ALIAS(wheel_odometry0)))
    };

    int n_sensors = 1;

    double data_array[1] = {};

    while (1) {

        LOG_DBG("");
        for (int i = 0; i < n_sensors; i++) {
            // default all data to zero
            struct sensor_value value = {};

            // get accel if device present
            if (dev[i] != NULL) {
                sensor_sample_fetch(dev[i]);
                sensor_channel_get(dev[i], SENSOR_CHAN_ROTATION, &value);
                LOG_DBG("rotation %d: %d.%06d", i, value.val1, value.val2);
            }

            for (int j = 0; j < n_sensors; j++) {
                data_array[i] = value.val1 + value.val2 * 1e-6;
            }
        }

        // select first wheel encoder for data for now: TODO implement voting
        double rotation = data_array[0];

        // publish mag to zbus
        synapse_msgs_WheelOdometry msg;
        msg.rotation = -rotation; // account for negative rotation of encoder
        zbus_chan_pub(&chan_out_wheel_odometry, &msg, K_NO_WAIT);

        // 200 Hz
        k_sleep(K_MSEC(5));
    }
    return 0;
}

K_THREAD_DEFINE(sense_wheel_odometry_thread, MY_STACK_SIZE,
    sense_wheel_odometry_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
