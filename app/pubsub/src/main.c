/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

// zephyr
#include <zephyr/logging/log.h>
#include <dds/cdr/dds_cdrstream.h>

// zros
#include <zros/zros_pub.h>
#include <zros/zros_pull_sub.h>
#include <zros/zros_push_sub.h>

// ros messages
#include <action_msgs/msg/goal_info.h>
#include <aerial_msgs/msg/test.h>
#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/pose.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <unique_identifier_msgs/msg/uuid.h>

// zros topic list
#include "topic_list.h"

#define MY_STACK_SIZE 1024
#define USLEEP 100000
#define PUBLISHER_COUNT 2
#define PULL_SUBSCRIBER_COUNT 2
#define PUSH_SUBSCRIBER_COUNT 2

#define CDR_SAFETY_MARGIN 12

LOG_MODULE_REGISTER(pubsub, CONFIG_PUBSUB_LOG_LEVEL);

static const uint8_t ros2_header[4] = {0x00, 0x01, 0x00, 0x00};

#if PUBLISHER_COUNT > 0
/********************************************************************
 * pub entry point
 ********************************************************************/
sensor_msgs__msg__Imu pub_msg_imu[PUBLISHER_COUNT];
sensor_msgs__msg__BatteryState pub_msg_battery_state[PUBLISHER_COUNT];

void * allocator_malloc(size_t size) {
    return NULL;
}

void * allocator_realloc(void *ptr, size_t new_size) {
    printf("reallocating");
    return NULL;
}

void allocator_free(void * pt) {
}

dds_cdrstream_allocator_t dds_allocator = {
    .free = allocator_free,
    .malloc = allocator_malloc,
    .realloc = allocator_realloc,
};

static void pub_entry_point(void* p0, void* p1, void* p2)
{
    rosidl_runtime_c__String s_data = {};
    s_data.capacity = 11;
    s_data.size = 11;
    s_data.data = "0123456789";
    std_msgs__msg__String s_msg;
    s_msg.data = s_data;

    static const size_t msg_size = sizeof(s_msg);
    size_t hdr_size = sizeof(ros2_header);
    char data[sizeof(s_msg)] = {};
    char buf[sizeof(s_msg) + 4 + CDR_SAFETY_MARGIN] = {};
    const uint32_t cdr_ops = DDS_OP_ADR;

    memcpy(data, &s_msg, sizeof(s_msg));

    // setup ostream
    dds_ostream_t os;
    os.m_buffer = buf;
    os.m_index = hdr_size;
    os.m_size = hdr_size + msg_size + CDR_SAFETY_MARGIN;
    os.m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_2;
    dds_stream_write(&os, &dds_allocator, data, &cdr_ops);

    // setup istream
    /*char buf_out[sizeof(s_msg) + 4 + CDR_SAFETY_MARGIN] = {};*/
    std_msgs__msg__String s_msg_out = {};
    dds_istream_t is;
    os.m_buffer = data;
    os.m_index = hdr_size;
    os.m_size = hdr_size + msg_size + CDR_SAFETY_MARGIN;
    os.m_xcdr_version = DDSI_RTPS_CDR_ENC_VERSION_2;
    dds_stream_read(&is, (char*)&s_msg_out, &dds_allocator, &cdr_ops);

    LOG_INF("str in %s", s_msg.data.data);
    LOG_INF("str out %s", s_msg_out.data.data);

    int id = (int)(p0);

    // zros_node_t node;
    // snprintf(node.name, sizeof(node.name), "pub %d", id);
    int rc = 0;
    sensor_msgs__msg__Imu* imu = &pub_msg_imu[id];
    sensor_msgs__msg__BatteryState* battery = &pub_msg_battery_state[id];

    zros_pub_t pub_imu;
    rc = zros_pub_init(&pub_imu, &topic_imu, imu, K_MSEC(1));
    if (rc != 0) {
        LOG_ERR("pub %d init failed %d\n", id, rc);
        return;
    }

    zros_pub_t pub_battery;
    rc = zros_pub_init(&pub_battery, &topic_battery_state, battery, K_MSEC(1));
    if (rc != 0) {
        LOG_ERR("pub %d init failed %d\n", id, rc);
        return;
    }

    while (1) {
        k_usleep(USLEEP);
        double t = (double)k_uptime_ticks() / CONFIG_SYS_CLOCK_TICKS_PER_SEC;

        // imu
        imu->angular_velocity.x = 1 + id + t;
        imu->angular_velocity.y = 2 + id + t;
        imu->angular_velocity.z = 3 + id + t;
        rc = zros_pub_update(&pub_imu, K_MSEC(1));
        if (rc != 0) {
            LOG_ERR("pub %d imu, rc: %d\n", id, rc);
        }

        // battery
        battery->voltage = 10.0;
        rc = zros_pub_update(&pub_battery, K_MSEC(1));
        if (rc != 0) {
            LOG_ERR("pub %d battery, rc: %d\n", id, rc);
        }

        /*LOG_INF("pub %d\n", id);*/
    }
}
#endif

#if PULL_SUBSCRIBER_COUNT > 0
/********************************************************************
 * pull sub entry point
 ********************************************************************/
sensor_msgs__msg__Imu pull_sub_msg_imu[PULL_SUBSCRIBER_COUNT];
static void pull_sub_entry_point(void* p0, void* p1, void* p2)
{
    int id = (int)p0;
    int rc = 0;
    sensor_msgs__msg__Imu* imu = &pull_sub_msg_imu[id];

    zros_pull_sub_t sub;
    rc = zros_pull_sub_init(&sub, &topic_imu, imu, K_MSEC(1));

    if (rc != 0) {
        LOG_ERR("pull sub %d init failed %d\n", id, rc);
        return;
    }

    aerial_msgs__msg__Test test;
    test.data = true;

    while (1) {
        k_usleep(USLEEP);
        rc = zros_pull_sub_update(&sub, K_MSEC(1));
        rosidl_runtime_c__String s;
        s.capacity = 10;
        s.data = "hello";
        s.size = 5;
        imu->header.frame_id = s;
        builtin_interfaces__msg__Time time;
        time.sec = 1;
        time.nanosec = 10;
        imu->header.stamp = time;
        if (rc != 0) {
            LOG_ERR("pull sub %d, rc: %d\n", id, rc);
        }
        /*LOG_INF("pull sub %d, %10.4f %10.4f %10.4f\n",*/
        /*id, imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);*/
    }
}
#endif

#if PUSH_SUBSCRIBER_COUNT > 0
/********************************************************************
 * push sub entry point
 ********************************************************************/
sensor_msgs__msg__Imu push_sub_msg_imu[PUSH_SUBSCRIBER_COUNT];
sensor_msgs__msg__BatteryState push_sub_msg_battery[PUSH_SUBSCRIBER_COUNT];

static void push_sub_entry_point(void* p0, void* p1, void* p2)
{
    int id = (int)p0;
    int rc = 0;

    builtin_interfaces__msg__Time msg;
    msg.sec = 1;
    msg.nanosec = 2;
    builtin_interfaces__msg__Time__Sequence seq;
    seq.capacity = 1;
    seq.data = &msg;
    seq.size = 1;

    rosidl_runtime_c__String s;
    s.data = "hello";
    s.capacity = 6;

    std_msgs__msg__String string_msg;
    string_msg.data = s;

    geometry_msgs__msg__Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    pose.position.x = 1;
    pose.position.y = 2;
    pose.position.z = 3;

    unique_identifier_msgs__msg__UUID uuid;
    unique_identifier_msgs__msg__UUID__init(&uuid);
    uuid.uuid[0] = 1;

    action_msgs__msg__GoalInfo goal;
    goal.goal_id = uuid;

    // imu
    sensor_msgs__msg__Imu* imu = &push_sub_msg_imu[id];
    zros_push_sub_t sub_imu;
    rc = zros_push_sub_init(&sub_imu, &topic_imu, imu, K_MSEC(1), 0.1);
    if (rc != 0) {
        LOG_INF("push sub %d imu, init failed %d\n", id, rc);
        return;
    }

    // battery
    sensor_msgs__msg__BatteryState* battery = &push_sub_msg_battery[id];
    zros_pull_sub_t sub_battery;
    rc = zros_pull_sub_init(&sub_battery, &topic_battery_state, battery, K_MSEC(1));
    if (rc != 0) {
        LOG_INF("push sub %d battery, init failed %d\n", id, rc);
        return;
    }

    struct k_poll_event events[1] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
            K_POLL_MODE_NOTIFY_ONLY,
            &sub_imu.data_ready),
    };

    while (1) {
        int rc = 0;
        rc = k_poll(events, 1, K_FOREVER);
        k_poll_signal_reset(&sub_imu.data_ready);
        if (rc != 0) {
            LOG_INF("polling error!\n");
        }

        rc = zros_push_sub_update(&sub_imu, K_MSEC(1));
        if (rc != 0) {
            LOG_INF("push sub %d imu update, rc: %d\n", id, rc);
        }

        rc = zros_pull_sub_update(&sub_battery, K_MSEC(1));
        if (rc != 0) {
            LOG_INF("pull sub %d battery update, rc: %d\n", id, rc);
        }
        /*
        LOG_INF("push sub %d, imu: %10.4f %10.4f %10.4f\n",
            id, imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
        LOG_INF("pull sub %d, battery: %10.4f\n", id, battery->voltage);
        */
    }
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
 * pull subscribers threads
 ********************************************************************/
#if PULL_SUBSCRIBER_COUNT > 0
K_THREAD_DEFINE(pull_sub0, MY_STACK_SIZE, pull_sub_entry_point,
    0, NULL, NULL, 4, 0, 0);
#endif

#if PULL_SUBSCRIBER_COUNT > 1
K_THREAD_DEFINE(pull_sub1, MY_STACK_SIZE, pull_sub_entry_point,
    1, NULL, NULL, 5, 0, 0);
#endif

#if PULL_SUBSCRIBER_COUNT > 2
K_THREAD_DEFINE(pull_sub2, MY_STACK_SIZE, pull_sub_entry_point,
    2, NULL, NULL, 6, 0, 0);
#endif

#if PULL_SUBSCRIBER_COUNT > 3
K_THREAD_DEFINE(pull_sub3, MY_STACK_SIZE, pull_sub_entry_point,
    3, NULL, NULL, 4, 0, 0);
#endif

#if PULL_SUBSCRIBER_COUNT > 4
K_THREAD_DEFINE(pull_sub4, MY_STACK_SIZE, pull_sub_entry_point,
    4, NULL, NULL, 5, 0, 0);
#endif

#if PULL_SUBSCRIBER_COUNT > 5
K_THREAD_DEFINE(pull_sub5, MY_STACK_SIZE, pull_sub_entry_point,
    5, NULL, NULL, 6, 0, 0);
#endif

/********************************************************************
 * push subscribers threads
 ********************************************************************/
#if PUSH_SUBSCRIBER_COUNT > 0
K_THREAD_DEFINE(push_sub0, MY_STACK_SIZE, push_sub_entry_point,
    0, NULL, NULL, 4, 0, 0);
#endif

#if PUSH_SUBSCRIBER_COUNT > 1
K_THREAD_DEFINE(push_sub1, MY_STACK_SIZE, push_sub_entry_point,
    1, NULL, NULL, 5, 0, 0);
#endif

#if PUSH_SUBSCRIBER_COUNT > 2
K_THREAD_DEFINE(push_sub2, MY_STACK_SIZE, push_sub_entry_point,
    2, NULL, NULL, 5, 0, 0);
#endif

#if PUSH_SUBSCRIBER_COUNT > 3
K_THREAD_DEFINE(push_sub3, MY_STACK_SIZE, push_sub_entry_point,
    3, NULL, NULL, 5, 0, 0);
#endif

#if PUSH_SUBSCRIBER_COUNT > 4
K_THREAD_DEFINE(push_sub4, MY_STACK_SIZE, push_sub_entry_point,
    4, NULL, NULL, 5, 0, 0);
#endif

#if PUSH_SUBSCRIBER_COUNT > 5
K_THREAD_DEFINE(push_sub5, MY_STACK_SIZE, push_sub_entry_point,
    5, NULL, NULL, 5, 0, 0);
#endif

// vi: ts=4 sw=4 et
