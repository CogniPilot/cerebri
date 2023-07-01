#ifndef SYNAPSE_ZBUS_CHANNELS_H
#define SYNAPSE_ZBUS_CHANNELS_H

#include <zephyr/zbus/zbus.h>

#include "synapse_protobuf/actuators.pb.h"
#include "synapse_protobuf/bezier_trajectory.pb.h"
#include "synapse_protobuf/imu.pb.h"
#include "synapse_protobuf/joy.pb.h"
#include "synapse_protobuf/nav_sat_fix.pb.h"
#include "synapse_protobuf/odometry.pb.h"
#include "synapse_protobuf/time.pb.h"
#include "synapse_protobuf/twist.pb.h"

ZBUS_CHAN_DECLARE(chan_in_actuators);
ZBUS_CHAN_DECLARE(chan_in_bezier_trajectory);
ZBUS_CHAN_DECLARE(chan_in_clock_offset);
ZBUS_CHAN_DECLARE(chan_in_cmd_vel);
ZBUS_CHAN_DECLARE(chan_in_imu);
ZBUS_CHAN_DECLARE(chan_in_joy);
ZBUS_CHAN_DECLARE(chan_in_nav_sat_fix);
ZBUS_CHAN_DECLARE(chan_in_odometry);

ZBUS_CHAN_DECLARE(chan_out_actuators);
ZBUS_CHAN_DECLARE(chan_out_odometry);

#endif // SYNAPSE_ZBUS_CHANNELS_H
