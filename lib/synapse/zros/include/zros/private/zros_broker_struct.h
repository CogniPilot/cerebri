/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_BROKER_STRUCT_H
#define ZROS_BROKER_STRUCT_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros broker
 ********************************************************************/
struct zros_topic;

struct zros_broker {
    struct k_mutex _lock;
    sys_slist_t _nodes; // list of nodes
    sys_slist_t _topics; // list of topics
};

#endif // ZROS_BROKER_STRUCT_H
// vi: ts=4 sw=4 et
