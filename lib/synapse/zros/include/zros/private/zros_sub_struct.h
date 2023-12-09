/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_SUB_STRUCT_H
#define ZROS_SUB_STRUCT_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros_sub struct
 ********************************************************************/
struct zros_topic;
struct zros_node;

struct zros_sub {
    sys_snode_t _topic_list_node;
    sys_snode_t _node_list_node;
    struct zros_topic* _topic;
    void* _data;
    struct k_poll_signal _data_ready;
    double _rate_limit_hz;
    int64_t _last_update_ticks;
    struct k_poll_event _event;
    struct zros_node* _node;
};

#endif // ZROS_SUB_STRUCT_H
// vi: ts=4 sw=4 et
