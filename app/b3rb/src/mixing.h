/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CEREBRI_B3RB_MIXING_H
#define CEREBRI_B3RB_MIXING_H

#include <synapse_topic_list.h>

void b3rb_set_actuators(synapse_pb_Actuators* msg, double turn_angle, double omega_fwd, bool armed);

#endif // CEREBRI_B3RB_MIXING_H
/* vi: ts=4 sw=4 et */
