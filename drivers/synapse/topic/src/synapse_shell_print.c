/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
#include "lib/core/common/casadi/common.h"
#include <cerebri/core/casadi.h>

#include "synapse_shell_print.h"

LOG_MODULE_DECLARE(zros_topic);

int snprintf_cat(char *buf, int n, char const *fmt, ...)
{
	if (n <= 0) {
		return n;
	}
	int result = 0;
	va_list args;
	va_start(args, fmt);
	result = vsnprintf(buf, n, fmt, args);
	va_end(args);
	return result;
}

int snprint_pwm(char *buf, size_t n, synapse_pb_Pwm *m)
{
	size_t offset = 0;
	if (m->has_timestamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->timestamp);
	}

	for (int i = 0; i < m->channel_count; i++) {
		offset += snprintf_cat(buf + offset, n - offset, "%4d: %10ld", i, m->channel[i]);
	}
	offset += snprintf_cat(buf + offset, n - offset, "\n");
	return offset;
}

int snprint_actuators(char *buf, size_t n, synapse_pb_Actuators *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	for (int i = 0; i < m->position_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "position [m]\n");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f\n", m->position[i]);
	}

	for (int i = 0; i < m->velocity_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "velocity [m/s]\n");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f\n", m->velocity[i]);
	}

	for (int i = 0; i < m->normalized_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "normalized\n");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f\n", m->normalized[i]);
	}
	return offset;
}

int snprint_altimeter(char *buf, size_t n, synapse_pb_Altimeter *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset,
			       "alt: %10.4f m, vel: %10.4f m/s, ref alt: %10.4f m\n",
			       m->vertical_position, m->vertical_velocity, m->vertical_reference);
	return offset;
}

int snprint_battery_state(char *buf, size_t n, synapse_pb_BatteryState *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset, "voltage: %10.4f V, current: %10.4f A\n",
			       m->voltage, m->current);
	return offset;
}

int snprint_bezier_curve(char *buf, size_t n, synapse_pb_BezierTrajectory_Curve *m)
{
	size_t offset = 0;
	for (int i = 0; i < m->x_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "x");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->x[i]);
	}
	if (m->x_count > 0) {
		offset += snprintf_cat(buf + offset, n - offset, "\n");
	}

	for (int i = 0; i < m->y_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "y");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->y[i]);
	}
	if (m->y_count > 0) {
		offset += snprintf_cat(buf + offset, n - offset, "\n");
	}

	for (int i = 0; i < m->z_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "z");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->z[i]);
	}
	if (m->z_count > 0) {
		offset += snprintf_cat(buf + offset, n - offset, "\n");
	}

	for (int i = 0; i < m->yaw_count; i++) {
		if (i == 0) {
			offset += snprintf_cat(buf + offset, n - offset, "yaw");
		}
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->yaw[i]);
	}
	if (m->yaw_count > 0) {
		offset += snprintf_cat(buf + offset, n - offset, "\n");
	}

	offset += snprintf_cat(buf + offset, n - offset, "time stop: %lld\n", m->time_stop);
	return offset;
}

int snprint_bezier_trajectory(char *buf, size_t n, synapse_pb_BezierTrajectory *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	offset += snprintf_cat(buf + offset, n - offset, "time start: %lld\n", m->time_start);

	for (int i = 0; i < m->curves_count; i++) {
		offset += snprintf_cat(buf + offset, n - offset, "curve: %d\n", i);
		offset += snprint_bezier_curve(buf + offset, n - offset, &m->curves[i]);
	}
	return offset;
}

int snprint_status(char *buf, size_t n, synapse_pb_Status *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(
		buf + offset, n - offset,
		"armed: %s\ninput source: %s\ntopic source: %s\nmode: %s\nsafety: %s\nfuel: %s\n"
		"fuel level: %0.2d\%\npower: %10.2fW\nmessage: %s\ninput status: %s\n"
		"request_seq: %10d\nrequest_rejected:%2d\n",
		armed_str(m->arming), input_source_str(m->input_source),
		topic_source_str(m->topic_source), mode_str(m->mode), status_safety_str(m->safety),
		fuel_str(m->fuel), m->fuel_percentage, (double)m->power, m->status_message,
		link_status_str(m->input_status), m->request_seq, m->request_rejected);
	return offset;
}

int snprint_imu(char *buf, size_t n, synapse_pb_Imu *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	if (m->has_angular_velocity) {
		offset += snprintf_cat(buf + offset, n - offset, "angular velocity [rad/s]\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->angular_velocity);
	}

	if (m->has_linear_acceleration) {
		offset += snprintf_cat(buf + offset, n - offset, "linear acceleration [m/s^2]\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->linear_acceleration);
	}

	if (m->has_orientation) {
		offset += snprintf_cat(buf + offset, n - offset, "orientation\n");
		offset += snprint_quaternion(buf + offset, n - offset, &m->orientation);
	}
	return offset;
}

int snprint_input(char *buf, size_t n, synapse_pb_Input *m)
{
	size_t offset = 0;
	if (m->has_timestamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->timestamp);
	}

	offset += snprintf_cat(buf + offset, n - offset, "\nchannels:\t");
	for (int i = 0; i < m->channel_count; i++) {
		offset += snprintf_cat(buf + offset, n - offset, "%10.4f\t", (double)m->channel[i]);
	}

	offset += snprintf_cat(buf + offset, n - offset, "\n");
	return offset;
}

int snprint_ledarray(char *buf, size_t n, synapse_pb_LEDArray *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	for (int i = 0; i < m->led_count; i++) {
		offset += snprintf_cat(buf + offset, n - offset, "index: %4d rgb: %4d %4d %4d\n",
				       m->led[i].index, m->led[i].r, m->led[i].g, m->led[i].b);
	}
	return offset;
}

int snprint_magnetic_field(char *buf, size_t n, synapse_pb_MagneticField *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	if (m->has_magnetic_field) {
		offset += snprint_vector3(buf + offset, n - offset, &m->magnetic_field);
	}

	return offset;
}

int snprint_navsatfix(char *buf, size_t n, synapse_pb_NavSatFix *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset, "lat: %10.7f deg\n", m->latitude);
	offset += snprintf_cat(buf + offset, n - offset, "lon: %10.7f deg\n", m->longitude);
	offset += snprintf_cat(buf + offset, n - offset, "alt: %10.7f m\n", m->altitude);
	return offset;
}

int snprint_odometry(char *buf, size_t n, synapse_pb_Odometry *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	if (m->has_pose) {
		offset += snprint_pose(buf + offset, n - offset, &m->pose);
	}

	if (m->has_twist) {
		offset += snprint_twist(buf + offset, n - offset, &m->twist);
	}
	offset += snprintf_cat(buf + offset, n - offset, "child frame: %s\n", m->child_frame_id);
	return offset;
}

int snprint_pose(char *buf, size_t n, synapse_pb_Pose *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}

	if (m->has_position) {
		offset += snprintf_cat(buf + offset, n - offset, "position\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->position);
	}

	if (m->has_orientation) {
		offset += snprintf_cat(buf + offset, n - offset, "orientation\n");
		offset += snprint_quaternion(buf + offset, n - offset, &m->orientation);
	}
	return offset;
}

int snprint_quaternion(char *buf, size_t n, synapse_pb_Quaternion *m)
{
	double q[4] = {m->w, m->x, m->y, m->z};
	double yaw, pitch, roll;
	double rad2deg = 180 / 3.14159;
	CASADI_FUNC_ARGS(quat_to_eulerB321)
	args[0] = q;
	res[0] = &yaw;
	res[1] = &pitch;
	res[2] = &roll;
	CASADI_FUNC_CALL(quat_to_eulerB321)
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset,
			       "yaw: %8.2f pitch: %8.2f roll: %8.2f [deg]\nw: %10.4f x: %10.4f y: "
			       "%10.4f z: %10.4f\n",
			       rad2deg * yaw, rad2deg * pitch, rad2deg * roll, m->w, m->x, m->y,
			       m->z);
	return offset;
}

int snprint_safety(char *buf, size_t n, synapse_pb_Safety *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset, "safety: %s\n", safety_str(m->status));
	return offset;
}

int snprint_timestamp(char *buf, size_t n, synapse_pb_Timestamp *m)
{
	return snprintf_cat(buf, n, "stamp: %lld.%09d\n", m->seconds, m->nanos);
}

int snprint_clock_offset(char *buf, size_t n, synapse_pb_ClockOffset *m)
{
	return snprintf_cat(buf, n, "stamp: %lld.%09d\n", m->offset.seconds, m->offset.nanos);
}

int snprint_twist(char *buf, size_t n, synapse_pb_Twist *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	if (m->has_angular) {
		offset += snprintf_cat(buf + offset, n - offset, "angular\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->angular);
	}
	if (m->has_linear) {
		offset += snprintf_cat(buf + offset, n - offset, "linear\n");
		offset += snprint_vector3(buf + offset, n - offset, &m->linear);
	}
	return offset;
}

int snprint_vector3(char *buf, size_t n, synapse_pb_Vector3 *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset, "x: %10.7f y: %10.7f z: %10.7f\n", m->x,
			       m->y, m->z);
	return offset;
}

int snprint_imu_q31_array(char *buf, size_t n, synapse_pb_ImuQ31Array *m)
{
	size_t offset = 0;
	offset += snprintf_cat(buf + offset, n - offset, "frame count: %d\n", m->frame_count);
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	for (int i = 0; i < m->frame_count; i++) {
		synapse_pb_ImuQ31Array_Frame *f = &m->frame[i];
		offset += snprintf_cat(buf + offset, n - offset,
				       "%11d %11d %11d %11d %11d %11d %11d %11d\n", f->delta_nanos,
				       f->accel_x, f->accel_y, f->accel_z, f->gyro_x, f->gyro_y,
				       f->gyro_z, f->temp);
	}
	return offset;
}

int snprint_wheel_odometry(char *buf, size_t n, synapse_pb_WheelOdometry *m)
{
	size_t offset = 0;
	if (m->has_stamp) {
		offset += snprint_timestamp(buf + offset, n - offset, &m->stamp);
	}
	offset += snprintf_cat(buf + offset, n - offset, "rotation: %10.4f\n", m->rotation);
	return offset;
}

// vi: ts=4 sw=4 et
