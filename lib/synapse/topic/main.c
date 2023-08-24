/*
 * Copyright (c) 2022 Rodrigo Peixoto <rodrigopex@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>

#include <synapse/zbus/channels.h>

LOG_MODULE_DECLARE(synapse_topic, LOG_LEVEL_INF);
static const struct shell* g_sh = NULL;
static const struct zbus_channel* g_chan_echo = NULL;
static int g_chan_msg_count = 0;
typedef enum ListenerMode { LISTENER_MODE_NONE,
    LISTENER_MODE_HZ,
    LISTENER_MODE_ECHO } listener_mode_t;
static listener_mode_t g_listener_mode = LISTENER_MODE_NONE;

void listener_synapse_topic_callback(const struct zbus_channel* chan);
ZBUS_LISTENER_DEFINE(listener_synapse_topic, listener_synapse_topic_callback);

void print_Time(synapse_msgs_Time* msg)
{
    shell_print(g_sh, "stamp: %lld.%09d", msg->sec, msg->nanosec);
}

void print_Vector3(synapse_msgs_Vector3* msg)
{
    shell_print(g_sh, "x: %10.4f y: %10.4f z: %10.4f", msg->x, msg->y, msg->z);
}

void print_Point(synapse_msgs_Point* msg)
{
    shell_print(g_sh, "x: %10.4f y: %10.4f z: %10.4f", msg->x, msg->y, msg->z);
}

void print_Quaternion(synapse_msgs_Quaternion* msg)
{
    shell_print(g_sh, "w: %10.4f x: %10.4f y: %10.4f z: %10.4f", msg->w, msg->x, msg->y, msg->z);
}

void print_Twist(synapse_msgs_Twist* msg)
{
    if (msg->has_angular) {
        shell_print(g_sh, "angular");
        print_Vector3(&msg->angular);
    }
    if (msg->has_linear) {
        shell_print(g_sh, "linear");
        print_Vector3(&msg->linear);
    }
}

void print_TwistWithCovariance(synapse_msgs_TwistWithCovariance* msg)
{
    if (msg->has_twist) {
        print_Twist(&msg->twist);
    }

    shell_print(g_sh, "covariance");
    for (int i = 0; i < msg->covariance_count; i++) {
        shell_print(g_sh, "%10.4f", msg->covariance[i]);
    }
}

void print_Pose(synapse_msgs_Pose* msg)
{
    shell_print(g_sh, "position");
    if (msg->has_position) {
        print_Point(&msg->position);
    }

    shell_print(g_sh, "orientation");
    if (msg->has_orientation) {
        print_Quaternion(&msg->orientation);
    }
}

void print_PoseWithCovariance(synapse_msgs_PoseWithCovariance* msg)
{
    if (msg->has_pose) {
        print_Pose(&msg->pose);
    }

    shell_print(g_sh, "covariance");
    for (int i = 0; i < msg->covariance_count; i++) {
        shell_print(g_sh, "%10.4f", msg->covariance[i]);
    }
}

void print_Header(synapse_msgs_Header* msg)
{
    if (msg->has_stamp) {
        print_Time(&msg->stamp);
    }
    shell_print(g_sh, "frame: %s", msg->frame_id);
    shell_print(g_sh, "seq: %d", msg->seq);
}

void print_Altimeter(synapse_msgs_Altimeter* msg)
{
    shell_print(g_sh, "alt: %10.4f m, vel: %10.4f m/s, ref alt: %10.4f m",
        msg->vertical_position, msg->vertical_velocity, msg->vertical_reference);
}

void print_BatteryState(synapse_msgs_BatteryState* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }
    shell_print(g_sh, "voltage: %10.4f V, current: %10.4f A",
        msg->voltage, msg->current);
}

void print_NavSatFix(synapse_msgs_NavSatFix* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }
    shell_print(g_sh, "lat: %10.7f deg", msg->latitude);
    shell_print(g_sh, "lon: %10.7f deg", msg->longitude);
    shell_print(g_sh, "alt: %10.7f m", msg->altitude);
}

void print_Actuators(synapse_msgs_Actuators* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    shell_print(g_sh, "position [m]");
    for (int i = 0; i < msg->position_count; i++) {
        shell_print(g_sh, "%10.4f", msg->position[i]);
    }

    shell_print(g_sh, "velocity [m/s]");
    for (int i = 0; i < msg->velocity_count; i++) {
        shell_print(g_sh, "%10.4f", msg->velocity[i]);
    }

    shell_print(g_sh, "normalized");
    for (int i = 0; i < msg->normalized_count; i++) {
        shell_print(g_sh, "%10.4f", msg->normalized[i]);
    }
}

void print_Imu(synapse_msgs_Imu* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    if (msg->has_angular_velocity) {
        shell_print(g_sh, "angular velocity [rad/s]");
        print_Vector3(&msg->angular_velocity);

        shell_print(g_sh, "covariance");
        for (int i = 0; i < msg->angular_velocity_covariance_count; i++) {
            shell_print(g_sh, "%10.4f", msg->angular_velocity_covariance[i]);
        }
    }

    if (msg->has_linear_acceleration) {
        shell_print(g_sh, "linear acceleration [m/s^2]");
        print_Vector3(&msg->linear_acceleration);

        shell_print(g_sh, "covariance");
        for (int i = 0; i < msg->linear_acceleration_covariance_count; i++) {
            shell_print(g_sh, "%10.4f", msg->linear_acceleration_covariance[i]);
        }
    }

    if (msg->has_orientation) {
        shell_print(g_sh, "orientation");
        print_Quaternion(&msg->orientation);
    }
}

void print_Odometry(synapse_msgs_Odometry* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    if (msg->has_pose) {
        print_PoseWithCovariance(&msg->pose);
    }

    if (msg->has_twist) {
        print_TwistWithCovariance(&msg->twist);
    }
    shell_print(g_sh, "child frame: %s", msg->child_frame_id);
}

void print_WheelOdometry(synapse_msgs_WheelOdometry* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    shell_print(g_sh, "rotation: %10.4f", msg->rotation);
}

void print_Joy(synapse_msgs_Joy* msg)
{
    shell_print(g_sh, "axes");
    for (int i = 0; i < msg->axes_count; i++) {
        shell_print(g_sh, "%10.4f", msg->axes[i]);
    }

    shell_print(g_sh, "buttons");
    for (int i = 0; i < msg->buttons_count; i++) {
        shell_print(g_sh, "%10d", msg->buttons[i]);
    }
}

void print_BezierCurve(synapse_msgs_BezierCurve* msg)
{
    shell_print(g_sh, "x");
    for (int i = 0; i < msg->x_count; i++) {
        shell_print(g_sh, "%10.4f", msg->x[i]);
    }
    shell_print(g_sh, "y");
    for (int i = 0; i < msg->y_count; i++) {
        shell_print(g_sh, "%10.4f", msg->y[i]);
    }
    shell_print(g_sh, "z");
    for (int i = 0; i < msg->z_count; i++) {
        shell_print(g_sh, "%10.4f", msg->z[i]);
    }
    shell_print(g_sh, "yaw");
    for (int i = 0; i < msg->yaw_count; i++) {
        shell_print(g_sh, "%10.4f", msg->yaw[i]);
    }
    shell_print(g_sh, "time stop: %lld", msg->time_stop);
}

void print_BezierTrajectory(synapse_msgs_BezierTrajectory* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    shell_print(g_sh, "time start: %lld", msg->time_start);

    for (int i = 0; i < msg->curves_count; i++) {
        shell_print(g_sh, "curve: %d", i);
        print_BezierCurve(&msg->curves[i]);
    }
}

void print_MagneticField(synapse_msgs_MagneticField* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }

    if (msg->has_magnetic_field) {
        print_Vector3(&msg->magnetic_field);
    }

    shell_print(g_sh, "covariance");
    for (int i = 0; i < msg->magnetic_field_covariance_count; i++) {
        shell_print(g_sh, "%10.4f", msg->magnetic_field_covariance[i]);
    }
}

void print_Fsm(synapse_msgs_Fsm* msg)
{
    if (msg->has_header) {
        print_Header(&msg->header);
    }
    shell_print(g_sh, "armed: %s, safety: %s, mode: %s",
        fsm_armed_str(msg->armed), fsm_safety_str(msg->safety), fsm_mode_str(msg->mode));
}

void listener_synapse_topic_callback(const struct zbus_channel* chan)
{
    // filter out channels we are not echoing
    if (chan != g_chan_echo) {
        return;
    }

    if (g_listener_mode == LISTENER_MODE_HZ) {
        g_chan_msg_count += 1;
    } else if (g_listener_mode == LISTENER_MODE_ECHO) {
        if (chan == &chan_out_nav_sat_fix) {
            synapse_msgs_NavSatFix* msg = (synapse_msgs_NavSatFix*)(chan->message);
            print_NavSatFix(msg);
        } else if (chan == &chan_in_actuators || chan == &chan_out_actuators) {
            synapse_msgs_Actuators* msg = (synapse_msgs_Actuators*)(chan->message);
            print_Actuators(msg);
        } else if (chan == &chan_in_clock_offset) {
            synapse_msgs_Time* msg = (synapse_msgs_Time*)(chan->message);
            print_Time(msg);
        } else if (chan == &chan_out_imu) {
            synapse_msgs_Imu* msg = (synapse_msgs_Imu*)(chan->message);
            print_Imu(msg);
        } else if (chan == &chan_in_odometry || chan == &chan_out_odometry) {
            synapse_msgs_Odometry* msg = (synapse_msgs_Odometry*)(chan->message);
            print_Odometry(msg);
        } else if (chan == &chan_out_altimeter) {
            synapse_msgs_Altimeter* msg = (synapse_msgs_Altimeter*)(chan->message);
            print_Altimeter(msg);
        } else if (chan == &chan_out_battery_state) {
            synapse_msgs_BatteryState* msg = (synapse_msgs_BatteryState*)(chan->message);
            print_BatteryState(msg);
        } else if (chan == &chan_out_magnetic_field) {
            synapse_msgs_MagneticField* msg = (synapse_msgs_MagneticField*)(chan->message);
            print_MagneticField(msg);
        } else if (chan == &chan_out_wheel_odometry) {
            synapse_msgs_WheelOdometry* msg = (synapse_msgs_WheelOdometry*)(chan->message);
            print_WheelOdometry(msg);
        } else if (chan == &chan_out_fsm) {
            synapse_msgs_Fsm* msg = (synapse_msgs_Fsm*)(chan->message);
            print_Fsm(msg);
        } else if (chan == &chan_in_joy) {
            synapse_msgs_Joy* msg = (synapse_msgs_Joy*)(chan->message);
            print_Joy(msg);
        } else if (chan == &chan_in_cmd_vel || chan == &chan_out_cmd_vel) {
            synapse_msgs_Twist msg = *(synapse_msgs_Twist*)(chan->message);
            print_Twist(&msg);
        } else if (chan == &chan_in_bezier_trajectory) {
            synapse_msgs_BezierTrajectory* msg = (synapse_msgs_BezierTrajectory*)(chan->message);
            print_BezierTrajectory(msg);
        } else {
            shell_print(g_sh, "%s printing not implemented", zbus_chan_name(chan));
        }

        // disable listener
        int ret = zbus_obs_set_enable(&listener_synapse_topic, false);
        if (ret != 0) {
            shell_print(g_sh, "could not disable observer: %d", ret);
        }
    }
}

static bool print_channel_name_iterator(const struct zbus_channel* chan)
{
    shell_print(g_sh, "%s", zbus_chan_name(chan) + 5);
    return true;
}

static int topic_list(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    g_sh = sh;
    zbus_iterate_over_channels(print_channel_name_iterator);
    return 0;
}

static int topic_echo(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    g_sh = sh;
    g_chan_echo = data;
    g_listener_mode = LISTENER_MODE_ECHO;
    int ret = zbus_obs_set_enable(&listener_synapse_topic, true);
    if (ret != 0) {
        shell_print(g_sh, "could not enable observer: %d", ret);
    }
    return 0;
}

static bool print_observer_name_iterator(const struct zbus_observer* obs)
{
    shell_print(g_sh, "      - %s", zbus_obs_name(obs));
    return true;
}

static int topic_info(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    const struct zbus_channel* chan = data;
    shell_print(sh, "channel %s:", zbus_chan_name(chan));
    shell_print(sh, "message size: %d", zbus_chan_msg_size(chan));
    shell_print(sh, "      observers:");
    zbus_iterate_over_observers(&print_observer_name_iterator);
    return 0;
}

static int topic_hz(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    g_sh = sh;
    g_chan_echo = data;
    g_chan_msg_count = 0;
    g_listener_mode = LISTENER_MODE_HZ;
    int ret = zbus_obs_set_enable(&listener_synapse_topic, true);
    if (ret != 0) {
        shell_print(g_sh, "could not enable observer: %d", ret);
    }
    k_msleep(2000);
    shell_print(g_sh, "Hz: %10.2f", g_chan_msg_count / 2.0f);
    return 0;
}

#define TOPIC_DICTIONARY()                                                          \
    (in_actuators, &chan_in_actuators, "in_actuators"),                             \
        (in_bezier_trajectory, &chan_in_bezier_trajectory, "in_bezier_trajectory"), \
        (in_clock_offset, &chan_in_clock_offset, "in_clock_offset"),                \
        (in_cmd_vel, &chan_in_cmd_vel, "in_cmd_vel"),                               \
        (in_joy, &chan_in_joy, "in_joy"),                                           \
        (in_nav_sat_fix, &chan_in_nav_sat_fix, "in_nav_sat_fix"),                   \
        (in_odometry, &chan_in_odometry, "in_odometry"),                            \
        (out_actuators, &chan_out_actuators, "out_actuators"),                      \
        (out_altimeter, &chan_out_altimeter, "out_altimeter"),                      \
        (out_battery_state, &chan_out_battery_state, "out_battery_state"),          \
        (out_cmd_vel, &chan_out_cmd_vel, "out_cmd_vel"),                            \
        (out_fsm, &chan_out_fsm, "out_fsm"),                                        \
        (out_imu, &chan_out_imu, "out_imu"),                                        \
        (out_magnetic_field, &chan_out_magnetic_field, "out_magnetic_field"),       \
        (out_nav_sat_fix, &chan_out_nav_sat_fix, "out_nav_sat_fix"),                \
        (out_odometry, &chan_out_odometry, "out_odometry"),                         \
        (out_wheel_odometry, &chan_out_wheel_odometry, "out_wheel_odometry")

/* Creating subcommands (level 2 command) dict for command "topic echo". */
SHELL_SUBCMD_DICT_SET_CREATE(sub_topic_echo, topic_echo, TOPIC_DICTIONARY());

/* Creating subcommands (level 2 command) dict for command "topic info". */
SHELL_SUBCMD_DICT_SET_CREATE(sub_topic_info, topic_info, TOPIC_DICTIONARY());

/* Creating subcommands (level 2 command) dict for command "topic hz". */
SHELL_SUBCMD_DICT_SET_CREATE(sub_topic_hz, topic_hz, TOPIC_DICTIONARY());

/* Creating subcommands (level 1 command) array for command "topic". */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_topic,
    SHELL_CMD(list, NULL, "list topics", topic_list),
    SHELL_CMD(info, &sub_topic_info, "show topic info", NULL),
    SHELL_CMD(echo, &sub_topic_echo, "echo topic data once", NULL),
    SHELL_CMD(hz, &sub_topic_hz, "topic pub rate", NULL),
    SHELL_SUBCMD_SET_END);

/* Creating root (level 0) command "topic" */
SHELL_CMD_REGISTER(topic, &sub_topic, "topic command", NULL);

/* add channel observer */
ZBUS_CHAN_ADD_OBS(chan_in_actuators, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_bezier_trajectory, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_clock_offset, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_cmd_vel, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_joy, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_nav_sat_fix, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_in_odometry, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_actuators, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_altimeter, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_battery_state, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_cmd_vel, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_fsm, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_imu, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_magnetic_field, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_nav_sat_fix, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_odometry, listener_synapse_topic, 1);
ZBUS_CHAN_ADD_OBS(chan_out_wheel_odometry, listener_synapse_topic, 1);

/* vi: ts=4 sw=4 et: */
