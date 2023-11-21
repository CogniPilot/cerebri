/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <zros/zros_node_manager.h>

/********************************************************************
 * zros node manager
 ********************************************************************/
void zros_node_manager_init(zros_node_t* node, const char* name)
{
    strncpy(node->name, name, sizeof(node->name));
    return;
};
// vi: ts=4 sw=4 et
