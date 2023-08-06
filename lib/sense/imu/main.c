/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <synapse/zbus/channels.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sense_imu, CONFIG_SENSE_IMU_LOG_LEVEL);

#define MY_STACK_SIZE 1280
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

    LOG_INF("imu found device %s", dev->name);
    return dev;
}

int sense_imu_entry_point(void)
{
    const struct device* const accel_dev[] = {
        sensor_check(DEVICE_DT_GET(DT_ALIAS(accel0))),
        sensor_check(DEVICE_DT_GET(DT_ALIAS(accel1))),
        sensor_check(DEVICE_DT_GET(DT_ALIAS(accel2)))
    };

    const struct device* const gyro_dev[] = {
        sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro0))),
        sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro1))),
        sensor_check(DEVICE_DT_GET(DT_ALIAS(gyro2)))
    };

    double accel_data_array[3][3] = {};
    double gyro_data_array[3][3] = {};

    while (1) {

        LOG_DBG("");
        for (int i = 0; i < 3; i++) {
            // default all data to zero
            struct sensor_value accel_value[3] = {};
            struct sensor_value gyro_value[3] = {};

            // get accel if device present
            if (accel_dev[i] != NULL) {
                sensor_sample_fetch(accel_dev[i]);
                sensor_channel_get(accel_dev[i], SENSOR_CHAN_ACCEL_XYZ, accel_value);
                LOG_DBG("accel %d: %d.%06d %d.%06d %d.%06d", i,
                    accel_value[0].val1, accel_value[0].val2,
                    accel_value[1].val1, accel_value[1].val2,
                    accel_value[2].val1, accel_value[2].val2);
            }

            // get gyro if device present
            if (gyro_dev[i] != NULL) {
                // don't resample if it is the same device as accel, want same timestamp
                if (gyro_dev[i] != accel_dev[i]) {
                    sensor_sample_fetch(gyro_dev[i]);
                }
                sensor_channel_get(gyro_dev[i], SENSOR_CHAN_GYRO_XYZ, gyro_value);
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
        synapse_msgs_Imu msg;
        strncpy(msg.header.frame_id, "map", sizeof(msg.header.frame_id) - 1);
        msg.angular_velocity.x = gyro[0];
        msg.angular_velocity.y = gyro[1];
        msg.angular_velocity.z = gyro[2];
        msg.linear_acceleration.x = accel[0];
        msg.linear_acceleration.y = accel[1];
        msg.linear_acceleration.z = accel[2];
        zbus_chan_pub(&chan_out_imu, &msg, K_NO_WAIT);

        // 200 Hz
        k_sleep(K_MSEC(5));
    }
    return 0;
}

K_THREAD_DEFINE(sense_imu_thread, MY_STACK_SIZE,
    sense_imu_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
