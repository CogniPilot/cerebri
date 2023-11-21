/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_PULL_SUB_H_
#define ZROS_PULL_SUB_H_

#include <zros/zros_topic.h>

/********************************************************************
 * zros_pull_sub
 ********************************************************************/
typedef struct zros_pull_sub_s {
    sys_snode_t list_node; // linked list node
    zros_topic_t* topic;
    void* data;
} zros_pull_sub_t;
int zros_pull_sub_init(zros_pull_sub_t* sub, zros_topic_t* topic, void* data,
    k_timeout_t timeout);
int zros_pull_sub_update(zros_pull_sub_t* sub, k_timeout_t timeout);

#endif // ZROS_PULL_SUB_H_
// vi: ts=4 sw=4 et
