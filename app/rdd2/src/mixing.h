/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CEREBRI_RDD2_MIXING_H
#define CEREBRI_RDD2_MIXING_H

#include <synapse_topic_list.h>

void rdd2_set_actuators(synapse_msgs_Actuators* msg, double roll, double pitch, double yaw, double thrust);

#endif // CEREBRI_RDD2_MIXING_H
/* vi: ts=4 sw=4 et */
