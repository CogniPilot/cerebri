/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_SUB_H
#define ZROS_SUB_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros_sub
 ********************************************************************/
// forward declarations
struct zros_sub;
struct zros_topic;
struct zros_node;

// public api
struct zros_node;
int zros_sub_init(struct zros_sub* sub, struct zros_node* node, struct zros_topic* topic, void* data,
    double rate_limit_hz);
int zros_sub_update(struct zros_sub* sub);
bool zros_sub_update_available(struct zros_sub* sub);
struct k_poll_event* zros_sub_get_event(struct zros_sub* sub);
void zros_sub_fini(struct zros_sub* sub);

#endif // ZROS_SUB_H
// vi: ts=4 sw=4 et
