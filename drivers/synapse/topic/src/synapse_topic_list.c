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
static const char *unhandled = "UNHANDLED";

void stamp_msg(synapse_pb_Timestamp *msg, int64_t ticks)
{
	int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 /
			  CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	msg->seconds = sec;
	msg->nanos = nanosec;
}

const char *input_source_str(synapse_pb_Status_InputSource src)
{
	if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_UNKNOWN) {
		return "unknown";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL) {
		return "radio_control";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET) {
		return "ethernet";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_CAN) {
		return "can";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_RESERVED_0) {
		return "reserved 0";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_RESERVED_1) {
		return "reserved 1";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_RESERVED_2) {
		return "reserved 2";
	} else if (src == synapse_pb_Status_InputSource_INPUT_SOURCE_RESERVED_3) {
		return "reserved 3";
	}
	return unhandled;
}

const char *topic_source_str(synapse_pb_Status_TopicSource src)
{
	if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_UNKNOWN) {
		return "unknown";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT) {
		return "input";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_LOCAL) {
		return "local";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_ETHERNET) {
		return "ethernet";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_CAN) {
		return "can";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_RESERVED_0) {
		return "reserved 0";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_RESERVED_1) {
		return "reserved 1";
	} else if (src == synapse_pb_Status_TopicSource_TOPIC_SOURCE_RESERVED_2) {
		return "reserved 2";
	}
	return unhandled;
}

const char *mode_str(synapse_pb_Status_Mode mode)
{
	if (mode == synapse_pb_Status_Mode_MODE_UNKNOWN) {
		return "unknown";
	} else if (mode == synapse_pb_Status_Mode_MODE_CALIBRATION) {
		return "calibration";
	} else if (mode == synapse_pb_Status_Mode_MODE_ACTUATORS) {
		return "manual";
	} else if (mode == synapse_pb_Status_Mode_MODE_ATTITUDE_RATE) {
		return "attitude rate";
	} else if (mode == synapse_pb_Status_Mode_MODE_ATTITUDE) {
		return "attitude";
	} else if (mode == synapse_pb_Status_Mode_MODE_ALTITUDE) {
		return "altitude";
	} else if (mode == synapse_pb_Status_Mode_MODE_POSITION) {
		return "position";
	} else if (mode == synapse_pb_Status_Mode_MODE_VELOCITY) {
		return "velocity";
	} else if (mode == synapse_pb_Status_Mode_MODE_ACCELERATION) {
		return "acceleration";
	} else if (mode == synapse_pb_Status_Mode_MODE_BEZIER) {
		return "bezier";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_0) {
		return "reserved 0";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_1) {
		return "reserved 1";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_2) {
		return "reserved 2";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_3) {
		return "reserved 3";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_4) {
		return "reserved 4";
	} else if (mode == synapse_pb_Status_Mode_MODE_RESERVED_5) {
		return "reserved 5";
	}
	return unhandled;
}

const char *armed_str(synapse_pb_Status_Arming arming)
{
	if (arming == synapse_pb_Status_Arming_ARMING_UNKNOWN) {
		return "unknown";
	} else if (arming == synapse_pb_Status_Arming_ARMING_ARMED) {
		return "armed";
	} else if (arming == synapse_pb_Status_Arming_ARMING_DISARMED) {
		return "disarmed";
	}
	return unhandled;
}

const char *safety_str(synapse_pb_Safety_Status safety)
{
	if (safety == synapse_pb_Safety_Status_SAFETY_UNKNOWN) {
		return "unknown";
	} else if (safety == synapse_pb_Safety_Status_SAFETY_SAFE) {
		return "safe";
	} else if (safety == synapse_pb_Safety_Status_SAFETY_UNSAFE) {
		return "unsafe";
	}
	return unhandled;
}

const char *status_safety_str(synapse_pb_Status_Safety safety)
{
	if (safety == synapse_pb_Status_Safety_SAFETY_UNKNOWN) {
		return "unknown";
	} else if (safety == synapse_pb_Status_Safety_SAFETY_SAFE) {
		return "safe";
	} else if (safety == synapse_pb_Status_Safety_SAFETY_UNSAFE) {
		return "unsafe";
	}
	return unhandled;
}

const char *link_status_str(synapse_pb_Status_LinkStatus status)
{
	if (status == synapse_pb_Status_LinkStatus_STATUS_UNKNOWN) {
		return "unknown";
	} else if (status == synapse_pb_Status_LinkStatus_STATUS_NOMINAL) {
		return "nominal";
	} else if (status == synapse_pb_Status_LinkStatus_STATUS_DISABLED) {
		return "disabled";
	} else if (status == synapse_pb_Status_LinkStatus_STATUS_LOSS) {
		return "loss";
	}
	return unhandled;
}

const char *fuel_str(synapse_pb_Status_Fuel fuel)
{
	if (fuel == synapse_pb_Status_Fuel_FUEL_UNKNOWN) {
		return "unknown";
	} else if (fuel == synapse_pb_Status_Fuel_FUEL_CRITICAL) {
		return "critical";
	} else if (fuel == synapse_pb_Status_Fuel_FUEL_LOW) {
		return "low";
	} else if (fuel == synapse_pb_Status_Fuel_FUEL_NOMINAL) {
		return "nominal";
	} else if (fuel == synapse_pb_Status_Fuel_FUEL_OVER_CAPACITY) {
		return "over capacity";
	}
	return unhandled;
}

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DEFINE(accel_ff, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(accel_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(actuators, synapse_pb_Actuators);
ZROS_TOPIC_DEFINE(altimeter, synapse_pb_Altimeter);
ZROS_TOPIC_DEFINE(angular_velocity_ff, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(angular_velocity_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(attitude_sp, synapse_pb_Quaternion);
ZROS_TOPIC_DEFINE(battery_state, synapse_pb_BatteryState);
ZROS_TOPIC_DEFINE(bezier_trajectory, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DEFINE(bezier_trajectory_ethernet, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DEFINE(clock_offset_ethernet, synapse_pb_ClockOffset);
ZROS_TOPIC_DEFINE(cmd_vel, synapse_pb_Twist);
ZROS_TOPIC_DEFINE(cmd_vel_ethernet, synapse_pb_Twist);
ZROS_TOPIC_DEFINE(force_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(imu, synapse_pb_Imu);
ZROS_TOPIC_DEFINE(imu_q31_array, synapse_pb_ImuQ31Array);
ZROS_TOPIC_DEFINE(input, synapse_pb_Input);
ZROS_TOPIC_DEFINE(input_ethernet, synapse_pb_Input);
ZROS_TOPIC_DEFINE(input_sbus, synapse_pb_Input);
ZROS_TOPIC_DEFINE(led_array, synapse_pb_LEDArray);
ZROS_TOPIC_DEFINE(magnetic_field, synapse_pb_MagneticField);
ZROS_TOPIC_DEFINE(moment_ff, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(moment_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(nav_sat_fix, synapse_pb_NavSatFix);
ZROS_TOPIC_DEFINE(odometry_estimator, synapse_pb_Odometry);
ZROS_TOPIC_DEFINE(odometry_ethernet, synapse_pb_Odometry);
ZROS_TOPIC_DEFINE(orientation_sp, synapse_pb_Quaternion);
ZROS_TOPIC_DEFINE(position_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(pwm, synapse_pb_Pwm);
ZROS_TOPIC_DEFINE(safety, synapse_pb_Safety);
ZROS_TOPIC_DEFINE(status, synapse_pb_Status);
ZROS_TOPIC_DEFINE(velocity_sp, synapse_pb_Vector3);
ZROS_TOPIC_DEFINE(wheel_odometry, synapse_pb_WheelOdometry);

static struct zros_topic *topic_list[] = {
	&topic_accel_ff,
	&topic_accel_sp,
	&topic_actuators,
	&topic_altimeter,
	&topic_angular_velocity_ff,
	&topic_angular_velocity_sp,
	&topic_attitude_sp,
	&topic_battery_state,
	&topic_bezier_trajectory,
	&topic_bezier_trajectory_ethernet,
	&topic_clock_offset_ethernet,
	&topic_cmd_vel,
	&topic_cmd_vel_ethernet,
	&topic_imu,
	&topic_imu_q31_array,
	&topic_input,
	&topic_input_ethernet,
	&topic_input_sbus,
	&topic_led_array,
	&topic_magnetic_field,
	&topic_moment_ff,
	&topic_nav_sat_fix,
	&topic_odometry_estimator,
	&topic_odometry_ethernet,
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
