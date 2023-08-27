/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_ACKERMANN_COMMON_H_
#define CONTROL_ACKERMANN_COMMON_H_

#include <cerebri/synapse/zbus/common.h>

void ackermann_set_actuators(synapse_msgs_Actuators* msg, double turn_angle, double omega_fwd);

#endif // CONTROL_ACKERMANN_COMMON_H_
/* vi: ts=4 sw=4 et */
