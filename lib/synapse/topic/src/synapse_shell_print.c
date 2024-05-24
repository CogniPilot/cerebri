/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <synapse_topic_list.h>

#include "synapse_shell_print.h"

int snprintf_cat(char* buf, int n, char const* fmt, ...)
{
    if (n <= 0)
        return n;
    int result = 0;
    va_list args;
    va_start(args, fmt);
    result = vsnprintf(buf, n, fmt, args);
    va_end(args);
    return result;
}

int snprint_actuators(char* buf, size_t n, synapse_msgs_Actuators* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
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

int snprint_altimeter(char* buf, size_t n, synapse_msgs_Altimeter* m)
{
    return snprintf_cat(buf, n, "alt: %10.4f m, vel: %10.4f m/s, ref alt: %10.4f m\n",
        m->vertical_position, m->vertical_velocity, m->vertical_reference);
}

int snprint_battery_state(char* buf, size_t n, synapse_msgs_BatteryState* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }
    offset += snprintf_cat(buf + offset, n - offset, "voltage: %10.4f V, current: %10.4f A\n",
        m->voltage, m->current);
    return offset;
}

int snprint_bezier_curve(char* buf, size_t n, synapse_msgs_BezierCurve* m)
{
    size_t offset = 0;
    for (int i = 0; i < m->x_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "x");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->x[i]);
    }
    for (int i = 0; i < m->y_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "y");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->y[i]);
    }
    for (int i = 0; i < m->z_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "z");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->z[i]);
    }
    for (int i = 0; i < m->yaw_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "yaw");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->yaw[i]);
    }
    offset += snprintf_cat(buf + offset, n - offset, "time stop: %lld\n", m->time_stop);
    return offset;
}

int snprint_bezier_trajectory(char* buf, size_t n, synapse_msgs_BezierTrajectory* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }

    offset += snprintf_cat(buf + offset, n - offset, "time start: %lld", m->time_start);

    for (int i = 0; i < m->curves_count; i++) {
        offset += snprintf_cat(buf + offset, n - offset, "curve: %d\n", i);
        offset += snprint_bezier_curve(buf + offset, n - offset, &m->curves[i]);
    }
    return offset;
}

int snprint_status(char* buf, size_t n, synapse_msgs_Status* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }
    offset += snprintf_cat(buf + offset, n - offset,
        "armed: %s\nmode: %s\nsafety: %s\nfuel: %s\n"
        "fuel level: %0.2d\%\npower: %10.2fW\nmessage: %s\njoy: %s\n"
        "request_seq: %10d\nrequest_rejected:%2d\n",
        armed_str(m->arming), mode_str(m->mode), status_safety_str(m->safety),
        fuel_str(m->fuel), m->fuel_percentage, (double)m->power, m->status_message,
        status_joy_str(m->joy), m->request_seq, m->request_rejected);
    return offset;
}

int snprint_header(char* buf, size_t n, synapse_msgs_Header* m)
{
    size_t offset = 0;
    if (m->has_stamp) {
        offset += snprint_time(buf + offset, n - offset, &m->stamp);
    }
    offset += snprintf_cat(buf + offset, n - offset, "frame: %s\n", m->frame_id);
    offset += snprintf_cat(buf + offset, n - offset, "seq: %d\n", m->seq);
    return offset;
}

int snprint_imu(char* buf, size_t n, synapse_msgs_Imu* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }

    if (m->has_angular_velocity) {
        offset += snprintf_cat(buf + offset, n - offset, "angular velocity [rad/s]\n");
        offset += snprint_vector3(buf + offset, n - offset, &m->angular_velocity);

        for (int i = 0; i < m->angular_velocity_covariance_count; i++) {
            if (i == 0) {
                offset += snprintf_cat(buf + offset, n - offset, "covariance\n");
            }
            offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->angular_velocity_covariance[i]);
        }
    }

    if (m->has_linear_acceleration) {
        offset += snprintf_cat(buf + offset, n - offset, "linear acceleration [m/s^2]\n");
        offset += snprint_vector3(buf + offset, n - offset, &m->linear_acceleration);

        for (int i = 0; i < m->linear_acceleration_covariance_count; i++) {
            if (i == 0) {
                offset += snprintf_cat(buf + offset, n - offset, "covariance\n");
            }
            offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->linear_acceleration_covariance[i]);
        }
    }

    if (m->has_orientation) {
        offset += snprintf_cat(buf + offset, n - offset, "orientation\n");
        offset += snprint_quaternion(buf + offset, n - offset, &m->orientation);
    }
    return offset;
}

int snprint_joy(char* buf, size_t n, synapse_msgs_Joy* m)
{
    size_t offset = 0;
    offset += snprintf_cat(buf + offset, n - offset, "\naxes:\t");
    for (int i = 0; i < m->axes_count; i++) {
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f\t", (double)m->axes[i]);
    }

    offset += snprintf_cat(buf + offset, n - offset, "\nbuttons:\t");
    for (int i = 0; i < m->buttons_count; i++) {
        offset += snprintf_cat(buf + offset, n - offset, "%10d\t", m->buttons[i]);
    }
    offset += snprintf_cat(buf + offset, n - offset, "\n");
    return offset;
}

int snprint_ledarray(char* buf, size_t n, synapse_msgs_LEDArray* m)
{
    size_t offset = 0;
    for (int i = 0; i < m->led_count; i++) {
        offset += snprintf_cat(buf + offset, n - offset,
            "index: %4d rgb: %4d %4d %4d\n",
            m->led[i].index,
            m->led[i].r,
            m->led[i].g,
            m->led[i].b);
    }
    return offset;
}

int snprint_magnetic_field(char* buf, size_t n, synapse_msgs_MagneticField* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }

    if (m->has_magnetic_field) {
        offset += snprint_vector3(buf + offset, n - offset, &m->magnetic_field);
    }

    for (int i = 0; i < m->magnetic_field_covariance_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "covariance\n");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f\n", m->magnetic_field_covariance[i]);
    }
    return offset;
}

int snprint_navsatfix(char* buf, size_t n, synapse_msgs_NavSatFix* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }
    offset += snprintf_cat(buf + offset, n - offset, "lat: %10.7f deg\n", m->latitude);
    offset += snprintf_cat(buf + offset, n - offset, "lon: %10.7f deg\n", m->longitude);
    offset += snprintf_cat(buf + offset, n - offset, "alt: %10.7f m\n", m->altitude);
    return offset;
}

int snprint_odometry(char* buf, size_t n, synapse_msgs_Odometry* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }

    if (m->has_pose) {
        offset += snprint_pose_with_covariance(buf + offset, n - offset, &m->pose);
    }

    if (m->has_twist) {
        offset += snprint_twist_with_covariance(buf + offset, n - offset, &m->twist);
    }
    offset += snprintf_cat(buf + offset, n - offset, "child frame: %s\n", m->child_frame_id);
    return offset;
}

int snprint_point(char* buf, size_t n, synapse_msgs_Point* m)
{
    return snprintf_cat(buf, 100, "x: %10.4f y: %10.4f z: %10.4f\n", m->x, m->y, m->z);
}

int snprint_pose(char* buf, size_t n, synapse_msgs_Pose* m)
{
    size_t offset = 0;
    if (m->has_position) {
        offset += snprintf_cat(buf + offset, n - offset, "position\n");
        offset += snprint_point(buf + offset, n - offset, &m->position);
    }

    if (m->has_orientation) {
        offset += snprintf_cat(buf + offset, n - offset, "orientation\n");
        offset += snprint_quaternion(buf + offset, n - offset, &m->orientation);
    }
    return offset;
}

int snprint_pose_with_covariance(char* buf, size_t n, synapse_msgs_PoseWithCovariance* m)
{
    size_t offset = 0;
    if (m->has_pose) {
        offset += snprint_pose(buf + offset, n - offset, &m->pose);
    }

    for (int i = 0; i < m->covariance_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "covariance\n");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f\n", m->covariance[i]);
    }
    return offset;
}

int snprint_quaternion(char* buf, size_t n, synapse_msgs_Quaternion* m)
{
    return snprintf_cat(buf, n, "w: %10.4f x: %10.4f y: %10.4f z: %10.4f\n",
        m->w, m->x, m->y, m->z);
}

int snprint_safety(char* buf, size_t n, synapse_msgs_Safety* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }
    offset += snprintf_cat(buf + offset, n - offset, "safety: %s\n",
        safety_str(m->status));
    return offset;
}

int snprint_time(char* buf, size_t n, synapse_msgs_Time* m)
{
    return snprintf_cat(buf, n, "stamp: %lld.%09d\n", m->sec, m->nanosec);
}

int snprint_twist(char* buf, size_t n, synapse_msgs_Twist* m)
{
    size_t offset = 0;
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

int snprint_twist_with_covariance(char* buf, size_t n, synapse_msgs_TwistWithCovariance* m)
{
    size_t offset = 0;
    if (m->has_twist) {
        offset += snprint_twist(buf + offset, n - offset, &m->twist);
    }

    for (int i = 0; i < m->covariance_count; i++) {
        if (i == 0) {
            offset += snprintf_cat(buf + offset, n - offset, "covariance\n");
        }
        offset += snprintf_cat(buf + offset, n - offset, "%10.4f", m->covariance[i]);
    }
    return offset;
}

int snprint_vector3(char* buf, size_t n, synapse_msgs_Vector3* m)
{
    return snprintf_cat(buf, n, "x: %10.4f y: %10.4f z: %10.4f\n", m->x, m->y, m->z);
}

int snprint_wheel_odometry(char* buf, size_t n, synapse_msgs_WheelOdometry* m)
{
    size_t offset = 0;
    if (m->has_header) {
        offset += snprint_header(buf + offset, n - offset, &m->header);
    }
    offset += snprintf_cat(buf + offset, n - offset, "rotation: %10.4f\n", m->rotation);
    return offset;
}

// vi: ts=4 sw=4 et
