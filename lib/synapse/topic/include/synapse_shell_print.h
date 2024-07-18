/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_snprint_H
#define SYNAPSE_snprint_H

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <synapse_topic_list.h>

typedef int snprint_t(char* buf, size_t n, void* msg);
int snprint_actuators(char* buf, size_t n, synapse_pb_Actuators* m);
int snprint_altimeter(char* buf, size_t n, synapse_pb_Altimeter* m);
int snprint_battery_state(char* buf, size_t n, synapse_pb_BatteryState* m);
int snprint_bezier_curve(char* buf, size_t n, synapse_pb_BezierCurve* m);
int snprint_bezier_trajectory(char* buf, size_t n, synapse_pb_BezierTrajectory* m);
int snprint_header(char* buf, size_t n, synapse_pb_Header* m);
int snprint_imu(char* buf, size_t n, synapse_pb_Imu* m);
int snprint_input(char* buf, size_t n, synapse_pb_Input* m);
int snprint_ledarray(char* buf, size_t n, synapse_pb_LEDArray* m);
int snprint_magnetic_field(char* buf, size_t n, synapse_pb_MagneticField* m);
int snprint_navsatfix(char* buf, size_t n, synapse_pb_NavSatFix* m);
int snprint_odometry(char* buf, size_t n, synapse_pb_Odometry* m);
int snprint_point(char* buf, size_t n, synapse_pb_Point* m);
int snprint_pose(char* buf, size_t n, synapse_pb_Pose* m);
int snprint_pose_with_covariance(char* buf, size_t n, synapse_pb_PoseWithCovariance* m);
int snprint_pwm(char* buf, size_t n, synapse_pb_Pwm* m);
int snprint_quaternion(char* buf, size_t n, synapse_pb_Quaternion* m);
int snprint_safety(char* buf, size_t n, synapse_pb_Safety* m);
int snprint_status(char* buf, size_t n, synapse_pb_Status* m);
int snprint_time(char* buf, size_t n, synapse_pb_Time* m);
int snprint_twist(char* buf, size_t n, synapse_pb_Twist* m);
int snprint_twist_with_covariance(char* buf, size_t n, synapse_pb_TwistWithCovariance* m);
int snprint_vector3(char* buf, size_t n, synapse_pb_Vector3* m);
int snprint_wheel_odometry(char* buf, size_t n, synapse_pb_WheelOdometry* m);
int snprint_rates_sp(char* buf, size_t n, synapse_pb_Vector3* m);

#endif // SYNAPSE_snprint_LIST_H
// vi: ts=4 sw=4 et
