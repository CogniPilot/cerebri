/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_PUB_STRUCT_H
#define ZROS_PUB_STRUCT_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros pub struct
 ********************************************************************/
struct zros_pub {
    sys_snode_t _topic_list_node;
    sys_snode_t _node_list_node;
    struct zros_topic* _topic;
    void* _data;
    struct zros_node* _node;
};

// vi: ts=4 sw=4 et
#endif // ZROS_PUB_STRUCT_H
