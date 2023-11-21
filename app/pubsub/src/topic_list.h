/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef TOPIC_LIST_H_
#define TOPIC_LIST_H_

#include "zros/zros_topic.h"
#include <zephyr/kernel.h>

/********************************************************************
 * topics
 ********************************************************************/
extern zros_topic_t topic_imu;
extern zros_topic_t topic_battery_state;

#endif // TOPIC_LIST_H_
// vi: ts=4 sw=4 et
