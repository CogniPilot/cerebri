/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include "zros/zros_topic.h"

#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>

/********************************************************************
 * topics
 ********************************************************************/

sensor_msgs__msg__Imu g_msg_imu = {};

zros_topic_t topic_imu = {
    .data = &g_msg_imu,
    .size = sizeof(g_msg_imu),
    .sem_read = Z_SEM_INITIALIZER(topic_imu.sem_read, 6, 6),
    .lock_write = Z_MUTEX_INITIALIZER(topic_imu.lock_write)
};

sensor_msgs__msg__BatteryState g_msg_battery = {};

zros_topic_t topic_battery_state = {
    .data = &g_msg_battery,
    .size = sizeof(g_msg_battery),
    .sem_read = Z_SEM_INITIALIZER(topic_battery_state.sem_read, 6, 6),
    .lock_write = Z_MUTEX_INITIALIZER(topic_battery_state.lock_write)
};

// vi: ts=4 sw=4 et
