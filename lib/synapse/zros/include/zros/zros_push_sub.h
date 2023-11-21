/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_PUSH_SUB_H_
#define ZROS_PUSH_SUB_H_

#include <zros/zros_topic.h>

/********************************************************************
 * zros_push_sub
 ********************************************************************/
typedef struct zros_push_sub_s {
    sys_snode_t list_node; // linked list node
    zros_topic_t* topic;
    void* data;
    struct k_poll_signal data_ready;
    struct k_poll_event events;
    double rate_limit_hz;
    int64_t last_update_ticks;
} zros_push_sub_t;
int zros_push_sub_init(zros_push_sub_t* sub, zros_topic_t* topic, void* data,
    k_timeout_t timeout, double rate_limit_hz);
int zros_push_sub_update(zros_push_sub_t* sub, k_timeout_t timeout);

#endif // ZROS_PUSH_SUB_H_
// vi: ts=4 sw=4 et
