/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_PUB_H
#define ZROS_PUB_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros pub
 ********************************************************************/
// forwad declarations
struct zros_topic;
struct zros_pub;
struct zros_node;

// public api
int zros_pub_init(struct zros_pub* pub, struct zros_node* node, struct zros_topic* topic, void* data);
int zros_pub_update(struct zros_pub* pub);
void zros_pub_fini(struct zros_pub* node);
void zros_pub_get_node(struct zros_pub* pub, struct zros_node** node);

// vi: ts=4 sw=4 et
#endif // ZROS_PUB_H
