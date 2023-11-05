#ifndef SYNAPSE_ZBUS_CHANNELS_H
#define SYNAPSE_ZBUS_CHANNELS_H

#include <zephyr/zbus/zbus.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/altimeter.pb.h>
#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/bezier_trajectory.pb.h>
#include <synapse_protobuf/fsm.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/joy.pb.h>
#include <synapse_protobuf/led_array.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/safety.pb.h>
#include <synapse_protobuf/time.pb.h>
#include <synapse_protobuf/twist.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>

//*******************************************************************
// (chan_in) channels from ROS computer to cerebri
//*******************************************************************
ZBUS_CHAN_DECLARE(chan_in_actuators);
ZBUS_CHAN_DECLARE(chan_in_bezier_trajectory);
ZBUS_CHAN_DECLARE(chan_in_clock_offset);
ZBUS_CHAN_DECLARE(chan_in_cmd_vel);
ZBUS_CHAN_DECLARE(chan_in_joy);
ZBUS_CHAN_DECLARE(chan_in_led_array);
ZBUS_CHAN_DECLARE(chan_in_nav_sat_fix);
ZBUS_CHAN_DECLARE(chan_in_odometry);

//*******************************************************************
// (chan_out) channels from cerebri to other nodes
// on cerebri or to ROS computer
//*******************************************************************
ZBUS_CHAN_DECLARE(chan_out_actuators);
ZBUS_CHAN_DECLARE(chan_out_actuators_manual);
ZBUS_CHAN_DECLARE(chan_out_altimeter);
ZBUS_CHAN_DECLARE(chan_out_battery_state);
ZBUS_CHAN_DECLARE(chan_out_cmd_vel);
ZBUS_CHAN_DECLARE(chan_out_fsm);
ZBUS_CHAN_DECLARE(chan_out_imu);
ZBUS_CHAN_DECLARE(chan_out_magnetic_field);
ZBUS_CHAN_DECLARE(chan_out_nav_sat_fix);
ZBUS_CHAN_DECLARE(chan_out_odometry);
ZBUS_CHAN_DECLARE(chan_out_safety);
ZBUS_CHAN_DECLARE(chan_out_wheel_odometry);

//*******************************************************************
// helper functions and definitions
//*******************************************************************
void stamp_header(synapse_msgs_Header* hdr, int64_t ticks);
const char* fsm_mode_str(synapse_msgs_Fsm_Mode mode);
const char* fsm_armed_str(synapse_msgs_Fsm_Armed armed);
const char* safety_str(synapse_msgs_Safety_Status safety);

enum {
    JOY_BUTTON_MANUAL = 0,
    JOY_BUTTON_AUTO = 1,
    JOY_BUTTON_CMD_VEL = 2,
    JOY_BUTTON_DISARM = 6,
    JOY_BUTTON_ARM = 7,
};

enum {
    JOY_AXES_THRUST = 1,
    JOY_AXES_PITCH = 2,
    JOY_AXES_ROLL = 3,
    JOY_AXES_YAW = 4,
};

#endif // SYNAPSE_ZBUS_CHANNELS_H
