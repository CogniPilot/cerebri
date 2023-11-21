/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_NODE_H_
#define ZROS_NODE_H_

#include <zros/zros_pub.h>
#include <zros/zros_topic.h>

/********************************************************************
 * zros node
 ********************************************************************/
typedef struct zros_node_s {
    char name[100];
    zros_topic_t* topic;
    void* data;
    sys_slist_t push_subs; // list of push subscriptions
    sys_slist_t pull_subs; // list of pull subscriptions
    sys_slist_t pubs; // list of publications
} zros_node_t;

void zros_node_init(zros_node_t* node, const char* name);
int zros_node_create_pub(zros_node_t* node, zros_pub_t* pub, zros_topic_t* topic, void* data, k_timeout_t timeout);

#endif // ZROS_NODE_H_
// vi: ts=4 sw=4 et
