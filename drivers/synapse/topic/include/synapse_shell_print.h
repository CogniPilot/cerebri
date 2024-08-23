/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_SHELL_PRINT_H
#define SYNAPSE_SHELL_PRINT_H

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <synapse_topic_list.h>

typedef int snprint_t(char *buf, size_t n, void *msg);

int snprint_actuators(char *buf, size_t n, synapse_pb_Actuators *m);
int snprint_altimeter(char *buf, size_t n, synapse_pb_Altimeter *m);
int snprint_battery_state(char *buf, size_t n, synapse_pb_BatteryState *m);
int snprint_bezier_curve(char *buf, size_t n, synapse_pb_BezierTrajectory_Curve *m);
int snprint_bezier_trajectory(char *buf, size_t n, synapse_pb_BezierTrajectory *m);
int snprint_clock_offset(char *buf, size_t n, synapse_pb_ClockOffset *m);
int snprint_imu(char *buf, size_t n, synapse_pb_Imu *m);
int snprint_imu_q31_array(char *buf, size_t n, synapse_pb_ImuQ31Array *m);
int snprint_input(char *buf, size_t n, synapse_pb_Input *m);
int snprint_ledarray(char *buf, size_t n, synapse_pb_LEDArray *m);
int snprint_magnetic_field(char *buf, size_t n, synapse_pb_MagneticField *m);
int snprint_navsatfix(char *buf, size_t n, synapse_pb_NavSatFix *m);
int snprint_odometry(char *buf, size_t n, synapse_pb_Odometry *m);
int snprint_pose(char *buf, size_t n, synapse_pb_Pose *m);
int snprint_pwm(char *buf, size_t n, synapse_pb_Pwm *m);
int snprint_quaternion(char *buf, size_t n, synapse_pb_Quaternion *m);
int snprint_rates_sp(char *buf, size_t n, synapse_pb_Vector3 *m);
int snprint_safety(char *buf, size_t n, synapse_pb_Safety *m);
int snprint_status(char *buf, size_t n, synapse_pb_Status *m);
int snprint_timestamp(char *buf, size_t n, synapse_pb_Timestamp *m);
int snprint_twist(char *buf, size_t n, synapse_pb_Twist *m);
int snprint_vector3(char *buf, size_t n, synapse_pb_Vector3 *m);
int snprint_wheel_odometry(char *buf, size_t n, synapse_pb_WheelOdometry *m);

#endif // SYNAPSE_SHELL_PRINT_H
// vi: ts=4 sw=4 et
