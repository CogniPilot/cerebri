/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_BROKER_H
#define ZROS_BROKER_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros node manager
 ********************************************************************/
// forward declarations
struct zros_broker;
struct zros_node;
struct zros_topic;

// public node api
typedef void zros_node_iterator_t(const struct zros_node* node, void* data);
int zros_broker_add_node(struct zros_node* node);
int zros_broker_remove_node(struct zros_node* node);
int zros_broker_iterate_nodes(zros_node_iterator_t* iter, void* data);

// public topic api
typedef void zros_topic_iterator_t(const struct zros_topic* topic, void* data);
int zros_broker_add_topic(struct zros_topic* topic);
int zros_broker_remove_topic(struct zros_topic* topic);
int zros_broker_iterate_topic(zros_topic_iterator_t* iter, void* data);

#endif // ZROS_broker_H
// vi: ts=4 sw=4 et
