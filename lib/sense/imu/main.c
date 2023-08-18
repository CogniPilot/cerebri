/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sense_imu, CONFIG_SENSE_IMU_LOG_LEVEL);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;

static const struct device* g_accel_dev[3];
static const struct device* g_gyro_dev[3];
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

    LOG_INF("imu found device %s", dev->name);
    return dev;
}

void imu_work_handler(struct k_work* work)
{
    double accel_data_array[3][3] = {};
    double gyro_data_array[3][3] = {};

    //        LOG_DBG("");
    for (int i = 0; i < 3; i++) {
        // default all data to zero
        struct sensor_value accel_value[3] = {};
        struct sensor_value gyro_value[3] = {};

        // get accel if device present
        if (g_accel_dev[i] != NULL) {
            sensor_sample_fetch(g_accel_dev[i]);
            sensor_channel_get(g_accel_dev[i], SENSOR_CHAN_ACCEL_XYZ, accel_value);
            LOG_DBG("accel %d: %d.%06d %d.%06d %d.%06d", i,
                accel_value[0].val1, accel_value[0].val2,
                accel_value[1].val1, accel_value[1].val2,
                accel_value[2].val1, accel_value[2].val2);
        }

        // get gyro if device present
        if (g_gyro_dev[i] != NULL) {
            // don't resample if it is the same device as accel, want same timestamp
            if (g_gyro_dev[i] != g_accel_dev[i]) {
                sensor_sample_fetch(g_gyro_dev[i]);
            }
            sensor_channel_get(g_gyro_dev[i], SENSOR_CHAN_GYRO_XYZ, gyro_value);
            LOG_DBG("gyro %d: %d.%06d %d.%06d %d.%06d", i,
                gyro_value[0].val1, gyro_value[0].val2,
                gyro_value[1].val1, gyro_value[1].val2,
                gyro_value[2].val1, gyro_value[2].val2);
        }

        for (int j = 0; j < 3; j++) {
            accel_data_array[i][j] = accel_value[j].val1 + accel_value[j].val2 * 1e-6;
            gyro_data_array[i][j] = gyro_value[j].val1 + gyro_value[j].val2 * 1e-6;
        }
    }

    // select first imu for data for now: TODO implement voting
    double accel[3] = {
        accel_data_array[0][0],
        accel_data_array[0][1],
        accel_data_array[0][2]
    };

    double gyro[3] = {
        gyro_data_array[0][0],
        gyro_data_array[0][1],
        gyro_data_array[0][2]
    };

    // publish imu to zbus
    synapse_msgs_Imu msg = synapse_msgs_Imu_init_default;
    msg.has_header = true;
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    msg.header.seq = g_seq++;
    msg.header.has_stamp = true;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    strncpy(msg.header.frame_id, "base_link", sizeof(msg.header.frame_id) - 1);
    msg.has_angular_velocity = true;
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    msg.has_linear_acceleration = true;
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];
    zbus_chan_pub(&chan_out_imu, &msg, K_NO_WAIT);
}

K_WORK_DEFINE(imu_work, imu_work_handler);

void imu_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &imu_work);
}

K_TIMER_DEFINE(imu_timer, imu_timer_handler, NULL);

int sense_imu_entry_point(void)
{
    g_accel_dev[0] = sensor_check(DEVICE_DT_GET(DT_ALIAS(accel0)));
    g_accel_dev[1] = sensor_check(DEVICE_DT_GET(DT_ALIAS(accel1)));
    g_accel_dev[2] = sensor_check(DEVICE_DT_GET(DT_ALIAS(accel2)));

    g_gyro_dev[0] = sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro0)));
    g_gyro_dev[1] = sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro1)));
    g_gyro_dev[2] = sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro2)));

    k_timer_start(&imu_timer, K_MSEC(5), K_MSEC(5));
    return 0;
}

K_THREAD_DEFINE(sense_imu, THREAD_STACK_SIZE,
    sense_imu_entry_point, NULL, NULL, NULL,
    THREAD_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
