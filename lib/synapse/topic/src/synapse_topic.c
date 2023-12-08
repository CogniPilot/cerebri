/*
 * Copyright (c) 2023 CogniPilot Foundation <cogni@cognipilot.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <stdio.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

LOG_MODULE_REGISTER(zros_topic);

#include "synapse_shell_print.h"
#include "synapse_topic_list.h"

#define TOPIC_DICTIONARY()                                                     \
    (actuators, &topic_actuators, "actuators"),                                \
        (actuators_manual, &topic_actuators_manual, "actuators_manual"),       \
        (altimeter, &topic_altimeter, "altimeter"),                            \
        (battery_state, &topic_battery_state, "battery_state"),                \
        (bezier_trajectory, &topic_bezier_trajectory, "bezier_trajectory"),    \
        (clock_offset, &topic_clock_offset, "clock_offset"),                   \
        (cmd_vel, &topic_cmd_vel, "cmd_vel"),                                  \
        (estimator_odometry, &topic_estimator_odometry, "estimator_odometry"), \
        (external_odometry, &topic_external_odometry, "external_odometry"),    \
        (imu, &topic_imu, "imu"),                                              \
        (joy, &topic_joy, "joy"),                                              \
        (led_array, &topic_led_array, "led_array"),                            \
        (magnetic_field, &topic_magnetic_field, "magnetic_field"),             \
        (nav_sat_fix, &topic_nav_sat_fix, "nav_sat_fix"),                      \
        (safety, &topic_safety, "safety"),                                     \
        (status, &topic_status, "status"),                                     \
        (wheel_odometry, &topic_wheel_odometry, "wheel_odometry")

int topic_count_hz(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo)
{
    struct zros_sub sub;
    struct zros_node node;
    zros_node_init(&node, "sub hz");
    zros_sub_init(&sub, &node, topic, msg, 1000);
    struct k_poll_event events[1] = {
        *zros_sub_get_event(&sub),
    };
    int cnt = 0;
    int64_t ticks_start = k_uptime_ticks();
    float elapsed = 0;
    float sample_period = 5.0;
    while (true) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(sample_period * 1e3));
        if (rc != 0) {
            char name[20];
            zros_topic_get_name(topic, name, sizeof(name));
            LOG_WRN("%s not published.", name);
        }

        if (zros_sub_update_available(&sub)) {
            zros_sub_update(&sub);
            cnt++;
        }
        elapsed = (float)(k_uptime_ticks() - ticks_start) / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
        if (elapsed >= sample_period)
            break;
    }
    zros_sub_fini(&sub);
    zros_node_fini(&node);
    float rate = cnt / elapsed;
    shell_print(sh, "%s%10.2f Hz", "", rate);
    return ZROS_OK;
}

static char buf[5192] = {};

int topic_echo(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo)
{
    struct zros_sub sub;
    struct zros_node node;
    zros_node_init(&node, "sub hz");
    zros_sub_init(&sub, &node, topic, msg, 1000);
    char name[20] = {};
    struct k_poll_event events[1] = {
        *zros_sub_get_event(&sub),
    };
    float sample_period = 2.0;
    int rc = 0;
    rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(sample_period * 1e3));
    zros_topic_get_name(topic, name, sizeof(name));
    if (rc != 0) {
        zros_topic_get_name(topic, name, sizeof(name));
        LOG_WRN("%s not published.", name);
        return -1;
    } else {
        if (!zros_sub_update_available(&sub)) {
            zros_topic_get_name(topic, name, sizeof(name));
            LOG_WRN("%s no update available.", name);
            return -1;
        } else {
            zros_sub_update(&sub);
            echo(buf, sizeof(buf), msg);
            printf("%s", buf);
            memset(buf, 0, sizeof(buf));
        }
    }
    zros_sub_fini(&sub);
    zros_node_fini(&node);
    return ZROS_OK;
}

typedef int msg_handler_t(const struct shell* sh, struct zros_topic* topic, void* msg, snprint_t* echo);

int handle_msg(const struct shell* sh, struct zros_topic* topic, msg_handler_t* handler)
{
    if (topic == &topic_actuators || topic == &topic_actuators_manual) {
        synapse_msgs_Actuators msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_actuators);
    } else if (topic == &topic_altimeter) {
        synapse_msgs_Altimeter msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_altimeter);
    } else if (topic == &topic_battery_state) {
        synapse_msgs_BatteryState msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_battery_state);
    } else if (topic == &topic_bezier_trajectory) {
        synapse_msgs_BezierTrajectory msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_bezier_trajectory);
    } else if (topic == &topic_clock_offset) {
        synapse_msgs_Time msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_time);
    } else if (topic == &topic_cmd_vel) {
        synapse_msgs_Twist msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_twist);
    } else if (topic == &topic_status) {
        synapse_msgs_Status msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_status);
    } else if (topic == &topic_imu) {
        synapse_msgs_Imu msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_imu);
    } else if (topic == &topic_joy) {
        synapse_msgs_Joy msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_joy);
    } else if (topic == &topic_led_array) {
        synapse_msgs_LEDArray msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_ledarray);
    } else if (topic == &topic_magnetic_field) {
        synapse_msgs_MagneticField msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_magnetic_field);
    } else if (topic == &topic_nav_sat_fix) {
        synapse_msgs_NavSatFix msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_navsatfix);
    } else if (topic == &topic_estimator_odometry || topic == &topic_external_odometry) {
        synapse_msgs_Odometry msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_odometry);
    } else if (topic == &topic_safety) {
        synapse_msgs_Safety msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_safety);
    } else if (topic == &topic_wheel_odometry) {
        synapse_msgs_WheelOdometry msg = {};
        return handler(sh, topic, &msg, (snprint_t*)&snprint_wheel_odometry);
    } else {
        char name[20];
        zros_topic_get_name(topic, name, sizeof(name));
        shell_print(sh, "%s not handled", name);
    }
    return ZROS_OK;
}

static int cmd_zros_topic_hz(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct zros_topic* topic = (struct zros_topic*)data;
    return handle_msg(sh, topic, &topic_count_hz);
}

static int cmd_zros_topic_echo(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct zros_topic* topic = (struct zros_topic*)data;
    return handle_msg(sh, topic, &topic_echo);
}

void topic_print_iterator(const struct zros_topic* topic, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[20];
    zros_topic_get_name(topic, name, sizeof(name));
    shell_print(sh, "%s", name);
}

static int cmd_zros_topic_list(const struct shell* sh,
    size_t argc, char** argv)
{
    zros_broker_iterate_topics(topic_print_iterator, (void*)sh);
    return ZROS_OK;
}

void node_print_iterator(const struct zros_node* node, void* data)
{
    const struct shell* sh = (const struct shell*)data;
    char name[30];
    zros_node_get_name(node, name, sizeof(name));
    shell_print(sh, "%s", name);
}

static int cmd_zros_node_list(const struct shell* sh,
    size_t argc, char** argv)
{
    zros_broker_iterate_nodes(node_print_iterator, (void*)sh);
    return ZROS_OK;
}

// level 2 (topic echo/hz/list)
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_echo, cmd_zros_topic_echo, TOPIC_DICTIONARY());
SHELL_SUBCMD_DICT_SET_CREATE(sub_zros_topic_hz, cmd_zros_topic_hz, TOPIC_DICTIONARY());
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_topic,
    SHELL_CMD(echo, &sub_zros_topic_echo, "Echo topic.", NULL),
    SHELL_CMD(hz, &sub_zros_topic_hz, "Check topic pub rate.", NULL),
    SHELL_CMD(list, NULL, "List topics.", cmd_zros_topic_list),
    SHELL_SUBCMD_SET_END);

// level 2 (node list)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros_node,
    SHELL_CMD(list, NULL, "List nodes.", cmd_zros_node_list),
    SHELL_SUBCMD_SET_END);

// level 1 (topic/node)
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zros,
    SHELL_CMD(topic, &sub_zros_topic, "Topic commands.", NULL),
    SHELL_CMD(node, &sub_zros_node, "Node commands.", NULL),
    SHELL_SUBCMD_SET_END);

// level 0 (zros)
SHELL_CMD_REGISTER(zros, &sub_zros, "ZROS Commands", NULL);

/* vi: ts=4 sw=4 et: */
