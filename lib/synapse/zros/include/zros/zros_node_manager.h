/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_NODE_MANAGER_H_
#define ZROS_NODE_MANAGER_H_

#include <zros/zros_node.h>

/********************************************************************
 * zros node manager
 ********************************************************************/
typedef struct zros_node_manger_s {
} zros_node_manager_t;

void zros_node_manager_init(zros_node_t* node, const char* name);

#endif // ZROS_NODE_MANAGER_H_
// vi: ts=4 sw=4 et
