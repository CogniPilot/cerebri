/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROL_DIFFDRIVE_MIXING_H_
#define CONTROL_DIFFDRIVE_MIXING_H_

#include <synapse_topic_list.h>

void elm4_set_actuators(synapse_msgs_Actuators* msg, double omega_fwd, double omega_turn);

#endif // CONTROL_DIFFDRIVE_MIXING_H_
/* vi: ts=4 sw=4 et */
