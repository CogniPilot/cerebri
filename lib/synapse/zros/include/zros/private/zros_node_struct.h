/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_NODE_STRUCT_H
#define ZROS_NODE_STRUCT_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros node
 ********************************************************************/
struct zros_node {
    const char* _name;
    sys_snode_t _broker_list_node;
    sys_slist_t _subs; // list of subscriptions
    sys_slist_t _pubs; // list of publications
    struct k_mutex _lock;
};

#endif // ZROS_NODE_STRUCT_H
// vi: ts=4 sw=4 et
