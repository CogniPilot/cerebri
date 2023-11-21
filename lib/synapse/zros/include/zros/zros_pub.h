/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_PUB_H_
#define ZROS_PUB_H_

#include <zros/zros_topic.h>

/********************************************************************
 * zros pub
 ********************************************************************/
typedef struct zros_pub_s {
    sys_snode_t list_node; // linked list node
    zros_topic_t* topic;
    void* data;
} zros_pub_t;
int zros_pub_init(zros_pub_t* pub, zros_topic_t* topic, void* data, k_timeout_t timeout);
int zros_pub_update(zros_pub_t* pub, k_timeout_t timeout);

// vi: ts=4 sw=4 et
#endif // ZROS_PUB_H_
