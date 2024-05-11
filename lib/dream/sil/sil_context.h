/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CEREBRI_DREAM_SIL_H
#define CEREBRI_DREAM_SIL_H

#include <signal.h>

#include <synapse_tinyframe/TinyFrame.h>

#include <synapse_protobuf/actuators.pb.h>
#include <synapse_protobuf/altimeter.pb.h>
#include <synapse_protobuf/battery_state.pb.h>
#include <synapse_protobuf/imu.pb.h>
#include <synapse_protobuf/led_array.pb.h>
#include <synapse_protobuf/magnetic_field.pb.h>
#include <synapse_protobuf/nav_sat_fix.pb.h>
#include <synapse_protobuf/odometry.pb.h>
#include <synapse_protobuf/sim_clock.pb.h>
#include <synapse_protobuf/wheel_odometry.pb.h>

#define RX_BUF_SIZE 4096

typedef struct context_s {
    const char* module_name;
    int sock;
    char rx_buf[RX_BUF_SIZE];
    pthread_t thread;
    bool clock_initialized;
    volatile sig_atomic_t shutdown;
    TinyFrame tf;
    synapse_msgs_SimClock sim_clock;
    synapse_msgs_Time clock_offset;
    synapse_msgs_NavSatFix nav_sat_fix;
    synapse_msgs_Imu imu;
    synapse_msgs_MagneticField magnetic_field;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Altimeter altimeter;
    synapse_msgs_WheelOdometry wheel_odometry;
    synapse_msgs_Odometry external_odometry;
    synapse_msgs_Actuators send_actuators;
    synapse_msgs_LEDArray send_led_array;
} sil_context_t;

#endif // CEREBRI_DREAM_SIL_H
// vi: ts=4 sw=4 et
