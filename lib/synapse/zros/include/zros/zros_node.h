/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_NODE_H
#define ZROS_NODE_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros node
 ********************************************************************/
// forward declarations
struct zros_node;
struct zros_sub;
struct zros_pub;

// public api
void zros_node_init(struct zros_node* node, const char* name);
int zros_node_add_sub(struct zros_node* node, struct zros_sub* sub);
int zros_node_add_pub(struct zros_node* node, struct zros_pub* pub);
int zros_node_get_name(const struct zros_node* node, char* buf, size_t n);
void zros_node_fini(struct zros_node* node);
void zros_node_append_to_list(struct zros_node* node, sys_slist_t* list);
void zros_node_remove_from_list(struct zros_node* node, sys_slist_t* list);

#endif // ZROS_NODE_H
// vi: ts=4 sw=4 et
