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

const char* command_source_str(synapse_msgs_Status_CommandSource src)
{
    if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_UNKNOWN) {
        return "unknown";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_ONBOARD) {
        return "onboard";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_OFFBOARD) {
        return "offboard";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_RESERVED_0) {
        return "reserved 0";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_RESERVED_1) {
        return "reserved 1";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_RESERVED_2) {
        return "reserved 2";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_RESERVED_3) {
        return "reserved 3";
    } else if (src == synapse_msgs_Status_CommandSource_COMMAND_SOURCE_RESERVED_4) {
        return "reserved 4";
    }
    return unhandled;
}

const char* mode_str(synapse_msgs_Status_Mode mode)
{
    if (mode == synapse_msgs_Status_Mode_MODE_UNKNOWN) {
        return "unknown";
    } else if (mode == synapse_msgs_Status_Mode_MODE_CALIBRATION) {
        return "calibration";
    } else if (mode == synapse_msgs_Status_Mode_MODE_MANUAL) {
        return "manual";
    } else if (mode == synapse_msgs_Status_Mode_MODE_ATTITUDE_RATE) {
        return "attitude rate";
    } else if (mode == synapse_msgs_Status_Mode_MODE_ATTITUDE) {
        return "attitude";
    } else if (mode == synapse_msgs_Status_Mode_MODE_ALTITUDE) {
        return "altitude";
    } else if (mode == synapse_msgs_Status_Mode_MODE_POSITION) {
        return "position";
    } else if (mode == synapse_msgs_Status_Mode_MODE_VELOCITY) {
        return "velocity";
    } else if (mode == synapse_msgs_Status_Mode_MODE_ACCELERATION) {
        return "acceleration";
    } else if (mode == synapse_msgs_Status_Mode_MODE_BEZIER) {
        return "bezier";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_0) {
        return "reserved 0";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_1) {
        return "reserved 1";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_2) {
        return "reserved 2";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_3) {
        return "reserved 3";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_4) {
        return "reserved 4";
    } else if (mode == synapse_msgs_Status_Mode_MODE_RESERVED_5) {
        return "reserved 5";
    }
    return unhandled;
}

const char* armed_str(synapse_msgs_Status_Arming arming)
{
    if (arming == synapse_msgs_Status_Arming_ARMING_UNKNOWN) {
        return "unknown";
    } else if (arming == synapse_msgs_Status_Arming_ARMING_ARMED) {
        return "armed";
    } else if (arming == synapse_msgs_Status_Arming_ARMING_DISARMED) {
        return "disarmed";
    }
    return unhandled;
}

const char* safety_str(synapse_msgs_Safety_Status safety)
{
    if (safety == synapse_msgs_Safety_Status_SAFETY_UNKNOWN) {
        return "unknown";
    } else if (safety == synapse_msgs_Safety_Status_SAFETY_SAFE) {
        return "safe";
    } else if (safety == synapse_msgs_Safety_Status_SAFETY_UNSAFE) {
        return "unsafe";
    }
    return unhandled;
}

const char* status_safety_str(synapse_msgs_Status_Safety safety)
{
    if (safety == synapse_msgs_Status_Safety_SAFETY_UNKNOWN) {
        return "unknown";
    } else if (safety == synapse_msgs_Status_Safety_SAFETY_SAFE) {
        return "safe";
    } else if (safety == synapse_msgs_Status_Safety_SAFETY_UNSAFE) {
        return "unsafe";
    }
    return unhandled;
}

const char* status_joy_str(synapse_msgs_Status_Joy joy)
{
    if (joy == synapse_msgs_Status_Joy_JOY_UNKNOWN) {
        return "unknown";
    } else if (joy == synapse_msgs_Status_Joy_JOY_NOMINAL) {
        return "nominal";
    } else if (joy == synapse_msgs_Status_Joy_JOY_LOSS) {
        return "loss";
    }
    return unhandled;
}

const char* fuel_str(synapse_msgs_Status_Fuel fuel)
{
    if (fuel == synapse_msgs_Status_Fuel_FUEL_UNKNOWN) {
        return "unknown";
    } else if (fuel == synapse_msgs_Status_Fuel_FUEL_CRITICAL) {
        return "critical";
    } else if (fuel == synapse_msgs_Status_Fuel_FUEL_LOW) {
        return "low";
    } else if (fuel == synapse_msgs_Status_Fuel_FUEL_NOMINAL) {
        return "nominal";
    } else if (fuel == synapse_msgs_Status_Fuel_FUEL_OVER_CAPACITY) {
        return "over capacity";
    }
    return unhandled;
}

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(accel_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(actuators, synapse_msgs_Actuators);
ZROS_TOPIC_DEFINE(altimeter, synapse_msgs_Altimeter);
ZROS_TOPIC_DEFINE(angular_velocity_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(attitude_sp, synapse_msgs_Quaternion);
ZROS_TOPIC_DEFINE(battery_state, synapse_msgs_BatteryState);
ZROS_TOPIC_DEFINE(bezier_trajectory, synapse_msgs_BezierTrajectory);
ZROS_TOPIC_DEFINE(cmd_vel, synapse_msgs_Twist);
ZROS_TOPIC_DEFINE(estimator_odometry, synapse_msgs_Odometry);
ZROS_TOPIC_DEFINE(force_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(imu, synapse_msgs_Imu);
ZROS_TOPIC_DEFINE(joy, synapse_msgs_Joy);
ZROS_TOPIC_DEFINE(led_array, synapse_msgs_LEDArray);
ZROS_TOPIC_DEFINE(magnetic_field, synapse_msgs_MagneticField);
ZROS_TOPIC_DEFINE(moment_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(nav_sat_fix, synapse_msgs_NavSatFix);
ZROS_TOPIC_DEFINE(offboard_bezier_trajectory, synapse_msgs_BezierTrajectory);
ZROS_TOPIC_DEFINE(offboard_clock_offset, synapse_msgs_Time);
ZROS_TOPIC_DEFINE(offboard_cmd_vel, synapse_msgs_Twist);
ZROS_TOPIC_DEFINE(offboard_joy, synapse_msgs_Joy);
ZROS_TOPIC_DEFINE(offboard_odometry, synapse_msgs_Odometry);
ZROS_TOPIC_DEFINE(orientation_sp, synapse_msgs_Quaternion);
ZROS_TOPIC_DEFINE(position_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(pwm, synapse_msgs_Pwm);
ZROS_TOPIC_DEFINE(safety, synapse_msgs_Safety);
ZROS_TOPIC_DEFINE(status, synapse_msgs_Status);
ZROS_TOPIC_DEFINE(velocity_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DEFINE(wheel_odometry, synapse_msgs_WheelOdometry);

static struct zros_topic* topic_list[] = {
    &topic_accel_sp,
    &topic_actuators,
    &topic_altimeter,
    &topic_angular_velocity_sp,
    &topic_attitude_sp,
    &topic_battery_state,
    &topic_bezier_trajectory,
    &topic_cmd_vel,
    &topic_estimator_odometry,
    &topic_imu,
    &topic_joy,
    &topic_led_array,
    &topic_magnetic_field,
    &topic_nav_sat_fix,
    &topic_offboard_bezier_trajectory,
    &topic_offboard_clock_offset,
    &topic_offboard_cmd_vel,
    &topic_offboard_joy,
    &topic_offboard_odometry,
    &topic_orientation_sp,
    &topic_position_sp,
    &topic_pwm,
    &topic_safety,
    &topic_status,
    &topic_velocity_sp,
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
