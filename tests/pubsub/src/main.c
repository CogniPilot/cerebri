/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

// zephyr
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

// zros
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 1024

#define USLEEP_PUB 50000

#define PUBLISHER_COUNT 10
#define SUBSCRIBER_COUNT 10

LOG_MODULE_REGISTER(pubsub, CONFIG_PUBSUB_LOG_LEVEL);

#if PUBLISHER_COUNT > 0
/********************************************************************
 * pub entry point
 ********************************************************************/
synapse_msgs_Imu pub_msg_imu[PUBLISHER_COUNT] = {};
synapse_msgs_BatteryState pub_msg_battery_state[PUBLISHER_COUNT] = {};

void* allocator_malloc(size_t size)
{
    return NULL;
}

void* allocator_realloc(void* ptr, size_t new_size)
{
    printf("reallocating");
    return NULL;
}

void allocator_free(void* pt)
{
}

static void pub_entry_point(void* p0, void* p1, void* p2)
{
    int id = (int)(p0);

    struct zros_node node = {};
    char name[20];
    snprintf(name, sizeof(name), "pub %u", id);
    zros_node_init(&node, name);

    // snprintf(node.name, sizeof(node.name), "pub %d", id);
    int rc = 0;
    synapse_msgs_Imu* imu = &pub_msg_imu[id];
    synapse_msgs_BatteryState* battery = &pub_msg_battery_state[id];

    struct zros_pub pub_imu;
    rc = zros_pub_init(&pub_imu, &node, &topic_imu, imu);
    if (rc != 0) {
        LOG_ERR("pub %d init failed %d\n", id, rc);
        return;
    }

    struct zros_pub pub_battery;
    rc = zros_pub_init(&pub_battery, &node, &topic_battery_state, battery);
    if (rc != 0) {
        LOG_ERR("pub %d init failed %d\n", id, rc);
        return;
    }

    while (true) {
        k_usleep(USLEEP_PUB);
        double t = (double)k_uptime_ticks() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

        // imu
        imu->has_angular_velocity = true;
        imu->angular_velocity.x = 1 + id + t;
        imu->angular_velocity.y = 2 + id + t;
        imu->angular_velocity.z = 3 + id + t;

        imu->has_linear_acceleration = true;
        imu->linear_acceleration.x = 1;
        imu->linear_acceleration.y = 2;
        imu->linear_acceleration.z = 3;
        rc = zros_pub_update(&pub_imu);
        if (rc != 0) {
            LOG_ERR("pub %d imu, rc: %d", id, rc);
        }

        // battery
        battery->voltage = 10.0;
        battery->current = 1.0;
        rc = zros_pub_update(&pub_battery);
        if (rc != 0) {
            LOG_ERR("pub %d battery, rc: %d", id, rc);
        }
        LOG_DBG("\npub %d", id);
    }

    zros_pub_fini(&pub_battery);
    zros_pub_fini(&pub_imu);
    zros_node_fini(&node);
}
#endif

#if SUBSCRIBER_COUNT > 0
/********************************************************************
 * sub entry point
 ********************************************************************/
synapse_msgs_Imu sub_msg_imu[SUBSCRIBER_COUNT];
synapse_msgs_BatteryState sub_msg_battery[SUBSCRIBER_COUNT];

static void sub_entry_point(void* p0, void* p1, void* p2)
{
    int id = (int)p0;
    int rc = 0;

    struct zros_node node = {};
    char name[20];
    snprintf(name, sizeof(name), "sub %u", id);
    zros_node_init(&node, name);

    // imu
    synapse_msgs_Imu* imu = &sub_msg_imu[id];
    struct zros_sub sub_imu;
    rc = zros_sub_init(&sub_imu, &node, &topic_imu, imu, 10);
    if (rc != 0) {
        LOG_DBG("sub %d imu, init failed %d", id, rc);
        return;
    }

    // battery
    synapse_msgs_BatteryState* battery = &sub_msg_battery[id];
    struct zros_sub sub_battery;
    rc = zros_sub_init(&sub_battery, &node, &topic_battery_state, battery, 1);
    if (rc != 0) {
        LOG_DBG("sub %d battery, init failed %d", id, rc);
        return;
    }

    struct k_poll_event events[2] = {
        *zros_sub_get_event(&sub_imu),
        *zros_sub_get_event(&sub_battery),
    };

    while (true) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
        if (rc != 0) {
            LOG_DBG("sub polling error! %d", rc);
        }

        // imu
        if (zros_sub_update_available(&sub_imu)) {
            zros_sub_update(&sub_imu);
            LOG_DBG("sub %d, imu: %10.4f %10.4f %10.4f",
                id, imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
        }

        // battery
        if (zros_sub_update_available(&sub_battery)) {
            zros_sub_update(&sub_battery);
            LOG_DBG("sub %d, battery: %10.4f %10.4f", id, battery->voltage, battery->current);
        }
    }

    zros_sub_fini(&sub_imu);
    zros_sub_fini(&sub_battery);
    zros_node_fini(&node);
}
#endif

/********************************************************************
 * publisher threads
 ********************************************************************/
#if PUBLISHER_COUNT > 0
K_THREAD_DEFINE(pub0, MY_STACK_SIZE, pub_entry_point,
    0, NULL, NULL, 1, 0, 0);
#endif

#if PUBLISHER_COUNT > 1
K_THREAD_DEFINE(pub1, MY_STACK_SIZE, pub_entry_point,
    1, NULL, NULL, 2, 0, 0);
#endif

#if PUBLISHER_COUNT > 2
K_THREAD_DEFINE(pub2, MY_STACK_SIZE, pub_entry_point,
    2, NULL, NULL, 3, 0, 0);
#endif

#if PUBLISHER_COUNT > 3
K_THREAD_DEFINE(pub3, MY_STACK_SIZE, pub_entry_point,
    3, NULL, NULL, 4, 0, 0);
#endif

#if PUBLISHER_COUNT > 4
K_THREAD_DEFINE(pub4, MY_STACK_SIZE, pub_entry_point,
    4, NULL, NULL, 1, 0, 0);
#endif

#if PUBLISHER_COUNT > 5
K_THREAD_DEFINE(pub5, MY_STACK_SIZE, pub_entry_point,
    5, NULL, NULL, 2, 0, 0);
#endif

/********************************************************************
 * subscribers threads
 ********************************************************************/
#if SUBSCRIBER_COUNT > 0
K_THREAD_DEFINE(sub0, MY_STACK_SIZE, sub_entry_point,
    0, NULL, NULL, 4, 0, 0);
#endif

#if SUBSCRIBER_COUNT > 1
K_THREAD_DEFINE(sub1, MY_STACK_SIZE, sub_entry_point,
    1, NULL, NULL, 5, 0, 0);
#endif

#if SUBSCRIBER_COUNT > 2
K_THREAD_DEFINE(sub2, MY_STACK_SIZE, sub_entry_point,
    2, NULL, NULL, 5, 0, 0);
#endif

#if SUBSCRIBER_COUNT > 3
K_THREAD_DEFINE(sub3, MY_STACK_SIZE, sub_entry_point,
    3, NULL, NULL, 5, 0, 0);
#endif

#if SUBSCRIBER_COUNT > 4
K_THREAD_DEFINE(sub4, MY_STACK_SIZE, sub_entry_point,
    4, NULL, NULL, 5, 0, 0);
#endif

#if SUBSCRIBER_COUNT > 5
K_THREAD_DEFINE(sub5, MY_STACK_SIZE, sub_entry_point,
    5, NULL, NULL, 5, 0, 0);
#endif

static int set_initial_log_level()
{
    log_filter_set(NULL, 0, log_source_id_get("pubsub"), LOG_LEVEL_INF);
    return 0;
}

SYS_INIT(set_initial_log_level, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// vi: ts=4 sw=4 et
