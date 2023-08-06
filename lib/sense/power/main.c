/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(sense_power, CONFIG_SENSE_POWER_LOG_LEVEL);

#define MY_STACK_SIZE 560
#define MY_PRIORITY 6

const struct device* const g_dev = DEVICE_DT_GET(DT_ALIAS(power0));

void publish_battery_data_zbus()
{

    struct sensor_value voltage, current;

    sensor_channel_get(g_dev, SENSOR_CHAN_VOLTAGE, &voltage);
    sensor_channel_get(g_dev, SENSOR_CHAN_CURRENT, &current);

    synapse_msgs_BatteryState msg_battery_state;

    msg_battery_state.voltage = sensor_value_to_double(&voltage);
    msg_battery_state.current = sensor_value_to_double(&current);

    zbus_chan_pub(&chan_out_battery_state, &msg_battery_state, K_NO_WAIT);
}

void sense_power_entry_point(void)
{
    int rc = 0;

    if (!device_is_ready(g_dev)) {
        LOG_ERR("Device %s is not ready", g_dev->name);
    }

    while (true) {
        k_sleep(K_MSEC(100));
        rc = sensor_sample_fetch(g_dev);
        if (rc) {
            LOG_ERR("Could not fetch sensor data");
            continue;
        }

        publish_battery_data_zbus();
    }
}

K_THREAD_DEFINE(sense_power_thread, MY_STACK_SIZE,
    sense_power_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
