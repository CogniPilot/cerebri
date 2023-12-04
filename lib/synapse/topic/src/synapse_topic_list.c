/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/slist.h>

#include <zros/private/zros_broker_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>

#include "synapse_topic_list.h"

//*******************************************************************
// helper functions
//*******************************************************************
static const char* unhandled = "UNHANDLED";

void stamp_header(synapse_msgs_Header* hdr, int64_t ticks)
{
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    hdr->has_stamp = true;
    hdr->stamp.sec = sec;
    hdr->stamp.nanosec = nanosec;
}

const char* fsm_mode_str(synapse_msgs_Fsm_Mode mode)
{
    if (mode == synapse_msgs_Fsm_Mode_UNKNOWN_MODE) {
        return "unknown";
    } else if (mode == synapse_msgs_Fsm_Mode_MANUAL) {
        return "manual";
    } else if (mode == synapse_msgs_Fsm_Mode_AUTO) {
        return "auto";
    } else if (mode == synapse_msgs_Fsm_Mode_CMD_VEL) {
        return "cmd_vel";
    }
    return unhandled;
}

const char* fsm_armed_str(synapse_msgs_Fsm_Armed armed)
{
    if (armed == synapse_msgs_Fsm_Armed_UNKNOWN_ARMING) {
        return "unknown";
    } else if (armed == synapse_msgs_Fsm_Armed_ARMED) {
        return "armed";
    } else if (armed == synapse_msgs_Fsm_Armed_DISARMED) {
        return "disarmed";
    }
    return unhandled;
}

const char* safety_str(synapse_msgs_Safety_Status safety)
{
    if (safety == synapse_msgs_Safety_Status_UNKNOWN) {
        return "unknown";
    } else if (safety == synapse_msgs_Safety_Status_SAFE) {
        return "safe";
    } else if (safety == synapse_msgs_Safety_Status_UNSAFE) {
        return "unsafe";
    }
    return unhandled;
}

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(actuators, synapse_msgs_Actuators);
ZROS_TOPIC_DEFINE(actuators_manual, synapse_msgs_Actuators);
ZROS_TOPIC_DEFINE(altimeter, synapse_msgs_Altimeter);
ZROS_TOPIC_DEFINE(battery_state, synapse_msgs_BatteryState);
ZROS_TOPIC_DEFINE(bezier_trajectory, synapse_msgs_BezierTrajectory);
ZROS_TOPIC_DEFINE(clock_offset, synapse_msgs_Time);
ZROS_TOPIC_DEFINE(cmd_vel, synapse_msgs_Twist);
ZROS_TOPIC_DEFINE(fsm, synapse_msgs_Fsm);
ZROS_TOPIC_DEFINE(imu, synapse_msgs_Imu);
ZROS_TOPIC_DEFINE(joy, synapse_msgs_Joy);
ZROS_TOPIC_DEFINE(led_array, synapse_msgs_LEDArray);
ZROS_TOPIC_DEFINE(magnetic_field, synapse_msgs_MagneticField);
ZROS_TOPIC_DEFINE(nav_sat_fix, synapse_msgs_NavSatFix);
ZROS_TOPIC_DEFINE(estimator_odometry, synapse_msgs_Odometry);
ZROS_TOPIC_DEFINE(external_odometry, synapse_msgs_Odometry);
ZROS_TOPIC_DEFINE(safety, synapse_msgs_Safety);
ZROS_TOPIC_DEFINE(wheel_odometry, synapse_msgs_WheelOdometry);

static struct zros_topic* topic_list[] = {
    &topic_actuators,
    &topic_actuators_manual,
    &topic_altimeter,
    &topic_battery_state,
    &topic_bezier_trajectory,
    &topic_clock_offset,
    &topic_cmd_vel,
    &topic_fsm,
    &topic_imu,
    &topic_joy,
    &topic_led_array,
    &topic_magnetic_field,
    &topic_nav_sat_fix,
    &topic_estimator_odometry,
    &topic_external_odometry,
    &topic_safety,
    &topic_wheel_odometry,
};

static int set_topic_list()
{
    for (size_t i = 0; i < ARRAY_SIZE(topic_list); i++) {
        zros_broker_add_topic(topic_list[i]);
    }
    return 0;
}

SYS_INIT(set_topic_list, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// vi: ts=4 sw=4 et
