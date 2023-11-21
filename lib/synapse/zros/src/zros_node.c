/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

/********************************************************************
 * zros node
 ********************************************************************/
void zros_node_init(zros_node_t* node, const char* name)
{
    strncpy(node->name, name, sizeof(node->name));
    return;
};

int zros_node_create_pub(zros_node_t* node, zros_pub_t* pub, zros_topic_t* topic, void* data, k_timeout_t timeout)
{
    // int zros_pub_init(zros_pub_t* pub, zros_topic_t* topic, void* data, k_timeout_t timeout)
    // strncpy(node->name, name, sizeof(node->name));
    return 0;
};

// vi: ts=4 sw=4 et
