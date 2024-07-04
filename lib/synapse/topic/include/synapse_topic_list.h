/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/altimeter.pb.h>
#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/bezier_trajectory.pb.h>
#include <synapse_protobuf/frame.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/input.pb.h>
#include <synapse_protobuf/led_array.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/pwm.pb.h>
#include <synapse_protobuf/quaternion.pb.h>
#include <synapse_protobuf/safety.pb.h>
#include <synapse_protobuf/status.pb.h>
#include <synapse_protobuf/time.pb.h>
#include <synapse_protobuf/twist.pb.h>
#include <synapse_protobuf/vector3.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>

/********************************************************************
 * helper
 ********************************************************************/
void stamp_header(synapse_msgs_Header* hdr, int64_t ticks);
const char* mode_str(synapse_msgs_Status_Mode mode);
const char* input_source_str(synapse_msgs_Status_InputSource src);
const char* topic_source_str(synapse_msgs_Status_TopicSource src);
const char* armed_str(synapse_msgs_Status_Arming arming);
const char* safety_str(synapse_msgs_Safety_Status safety);
const char* fuel_str(synapse_msgs_Status_Fuel fuel);
const char* status_safety_str(synapse_msgs_Status_Safety safety);
const char* link_status_str(synapse_msgs_Status_LinkStatus status);

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DECLARE(topic_accel_ff, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_accel_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_actuators, synapse_msgs_Actuators);
ZROS_TOPIC_DECLARE(topic_altimeter, synapse_msgs_Altimeter);
ZROS_TOPIC_DECLARE(topic_angular_velocity_ff, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_angular_velocity_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_attitude_sp, synapse_msgs_Quaternion);
ZROS_TOPIC_DECLARE(topic_battery_state, synapse_msgs_BatteryState);
ZROS_TOPIC_DECLARE(topic_bezier_trajectory, synapse_msgs_BezierTrajectory);
ZROS_TOPIC_DECLARE(topic_bezier_trajectory_ethernet, synapse_msgs_BezierTrajectory);
ZROS_TOPIC_DECLARE(topic_clock_offset_ethernet, synapse_msgs_Time);
ZROS_TOPIC_DECLARE(topic_cmd_vel, synapse_msgs_Twist);
ZROS_TOPIC_DECLARE(topic_cmd_vel_ethernet, synapse_msgs_Twist);
ZROS_TOPIC_DECLARE(topic_force_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_imu, synapse_msgs_Imu);
ZROS_TOPIC_DECLARE(topic_input, synapse_msgs_Input);
ZROS_TOPIC_DECLARE(topic_input_sbus, synapse_msgs_Input);
ZROS_TOPIC_DECLARE(topic_input_ethernet, synapse_msgs_Input);
ZROS_TOPIC_DECLARE(topic_led_array, synapse_msgs_LEDArray);
ZROS_TOPIC_DECLARE(topic_magnetic_field, synapse_msgs_MagneticField);
ZROS_TOPIC_DECLARE(topic_moment_ff, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_moment_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_nav_sat_fix, synapse_msgs_NavSatFix);
ZROS_TOPIC_DECLARE(topic_odometry_estimator, synapse_msgs_Odometry);
ZROS_TOPIC_DECLARE(topic_odometry_ethernet, synapse_msgs_Odometry);
ZROS_TOPIC_DECLARE(topic_orientation_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_position_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_pwm, synapse_msgs_Pwm);
ZROS_TOPIC_DECLARE(topic_safety, synapse_msgs_Safety);
ZROS_TOPIC_DECLARE(topic_status, synapse_msgs_Status);
ZROS_TOPIC_DECLARE(topic_velocity_sp, synapse_msgs_Vector3);
ZROS_TOPIC_DECLARE(topic_wheel_odometry, synapse_msgs_WheelOdometry);

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et
