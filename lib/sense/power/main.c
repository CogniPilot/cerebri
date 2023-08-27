/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <cerebri/synapse/zbus/channels.h>

LOG_MODULE_REGISTER(sense_power, CONFIG_CEREBRI_SENSE_POWER_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;
static const struct device* const g_dev = DEVICE_DT_GET(DT_ALIAS(power0));
static int32_t g_seq = 0;

void power_work_handler(struct k_work* work)
{
    int ret = sensor_sample_fetch(g_dev);
    if (ret) {
        LOG_ERR("Could not fetch sensor data");
        return;
    }
    struct sensor_value voltage, current;

    sensor_channel_get(g_dev, SENSOR_CHAN_VOLTAGE, &voltage);
    sensor_channel_get(g_dev, SENSOR_CHAN_CURRENT, &current);

    synapse_msgs_BatteryState msg = synapse_msgs_BatteryState_init_default;

    msg.has_header = true;
    stamp_header(&msg.header, k_uptime_ticks());
    msg.header.seq = g_seq++;
    strncpy(msg.header.frame_id, "base_link", sizeof(msg.header.frame_id) - 1);

    msg.voltage = sensor_value_to_double(&voltage);
    msg.current = sensor_value_to_double(&current);

    zbus_chan_pub(&chan_out_battery_state, &msg, K_NO_WAIT);
}

K_WORK_DEFINE(power_work, power_work_handler);

void power_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &power_work);
}

K_TIMER_DEFINE(power_timer, power_timer_handler, NULL);

int sense_power_entry_point(void)
{
    g_seq = 0;
    if (!device_is_ready(g_dev)) {
        LOG_ERR("Device %s is not ready", g_dev->name);
    }

    k_timer_start(&power_timer, K_MSEC(100), K_MSEC(100));
    return 0;
}

K_THREAD_DEFINE(sense_power, MY_STACK_SIZE,
    sense_power_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
