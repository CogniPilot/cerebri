/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_TOPIC_LIST_H
#define SYNAPSE_TOPIC_LIST_H

#include <zros/zros_topic.h>

#include <synapse_pb/actuators.pb.h>
#include <synapse_pb/altimeter.pb.h>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/bezier_trajectory.pb.h>
#include <synapse_pb/frame.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/input.pb.h>
#include <synapse_pb/led_array.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/pwm.pb.h>
#include <synapse_pb/quaternion.pb.h>
#include <synapse_pb/safety.pb.h>
#include <synapse_pb/status.pb.h>
#include <synapse_pb/twist.pb.h>
#include <synapse_pb/vector3.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

/********************************************************************
 * helper
 ********************************************************************/
void stamp_msg(synapse_pb_Timestamp *hdr, int64_t ticks);
const char *mode_str(synapse_pb_Status_Mode mode);
const char *input_source_str(synapse_pb_Status_InputSource src);
const char *topic_source_str(synapse_pb_Status_TopicSource src);
const char *armed_str(synapse_pb_Status_Arming arming);
const char *safety_str(synapse_pb_Safety_Status safety);
const char *fuel_str(synapse_pb_Status_Fuel fuel);
const char *status_safety_str(synapse_pb_Status_Safety safety);
const char *link_status_str(synapse_pb_Status_LinkStatus status);

/********************************************************************
 * topics
 ********************************************************************/
ZROS_TOPIC_DECLARE(topic_accel_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_accel_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_actuators, synapse_pb_Actuators);
ZROS_TOPIC_DECLARE(topic_altimeter, synapse_pb_Altimeter);
ZROS_TOPIC_DECLARE(topic_angular_velocity_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_angular_velocity_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_attitude_sp, synapse_pb_Quaternion);
ZROS_TOPIC_DECLARE(topic_battery_state, synapse_pb_BatteryState);
ZROS_TOPIC_DECLARE(topic_bezier_trajectory, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DECLARE(topic_bezier_trajectory_ethernet, synapse_pb_BezierTrajectory);
ZROS_TOPIC_DECLARE(topic_clock_offset_ethernet, synapse_pb_Time);
ZROS_TOPIC_DECLARE(topic_cmd_vel, synapse_pb_Twist);
ZROS_TOPIC_DECLARE(topic_cmd_vel_ethernet, synapse_pb_Twist);
ZROS_TOPIC_DECLARE(topic_force_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_imu, synapse_pb_Imu);
ZROS_TOPIC_DECLARE(topic_imu_q31_array, synapse_pb_ImuQ31Array);
ZROS_TOPIC_DECLARE(topic_input, synapse_pb_Input);
ZROS_TOPIC_DECLARE(topic_input_sbus, synapse_pb_Input);
ZROS_TOPIC_DECLARE(topic_input_ethernet, synapse_pb_Input);
ZROS_TOPIC_DECLARE(topic_led_array, synapse_pb_LEDArray);
ZROS_TOPIC_DECLARE(topic_magnetic_field, synapse_pb_MagneticField);
ZROS_TOPIC_DECLARE(topic_moment_ff, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_moment_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_nav_sat_fix, synapse_pb_NavSatFix);
ZROS_TOPIC_DECLARE(topic_odometry_estimator, synapse_pb_Odometry);
ZROS_TOPIC_DECLARE(topic_odometry_ethernet, synapse_pb_Odometry);
ZROS_TOPIC_DECLARE(topic_orientation_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_position_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_pwm, synapse_pb_Pwm);
ZROS_TOPIC_DECLARE(topic_safety, synapse_pb_Safety);
ZROS_TOPIC_DECLARE(topic_status, synapse_pb_Status);
ZROS_TOPIC_DECLARE(topic_velocity_sp, synapse_pb_Vector3);
ZROS_TOPIC_DECLARE(topic_wheel_odometry, synapse_pb_WheelOdometry);

#endif // SYNAPSE_TOPIC_LIST_H_
// vi: ts=4 sw=4 et
