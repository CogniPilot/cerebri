/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CEREBRI_DREAM_SIL_H
#define CEREBRI_DREAM_SIL_H

#include <signal.h>

#include <synapse_pb/actuators.pb.h>
#include <synapse_pb/altimeter.pb.h>
#include <synapse_pb/battery_state.pb.h>
#include <synapse_pb/imu.pb.h>
#include <synapse_pb/led_array.pb.h>
#include <synapse_pb/magnetic_field.pb.h>
#include <synapse_pb/nav_sat_fix.pb.h>
#include <synapse_pb/odometry.pb.h>
#include <synapse_pb/sim_clock.pb.h>
#include <synapse_pb/wheel_odometry.pb.h>

#define RX_BUF_SIZE 4096
#define TX_BUF_SIZE 4096

typedef struct context_s {
    const char* module_name;
    int sock;
    pthread_t thread;
    bool clock_initialized;
    volatile sig_atomic_t shutdown;
    synapse_pb_SimClock sim_clock;
    synapse_pb_Time clock_offset;
    synapse_pb_Actuators actuators;
    synapse_pb_LEDArray led_array;
} sil_context_t;

#endif // CEREBRI_DREAM_SIL_H
// vi: ts=4 sw=4 et
