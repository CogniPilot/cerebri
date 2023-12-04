/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_broker.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

static const k_timeout_t g_node_timeout = K_MSEC(1);

/********************************************************************
 * zros node
 ********************************************************************/
void zros_node_init(struct zros_node* node, const char* name)
{
    node->_name = name;
    node->_list_node.next = NULL;
    sys_slist_init(&node->_subs);
    sys_slist_init(&node->_pubs);
    k_mutex_init(&node->_lock);
    zros_broker_add_node(node);
};

int _zros_node_lock(struct zros_node* node)
{
    return k_mutex_lock(&node->_lock, g_node_timeout);
}

void _zros_node_unlock(struct zros_node* node)
{
    k_mutex_unlock(&node->_lock);
}

int zros_node_add_sub(struct zros_node* node, struct zros_sub* sub)
{
    ZROS_RC(_zros_node_lock(node), return rc);
    sys_slist_append(&node->_subs, &sub->_list_node);
    _zros_node_unlock(node);
    return ZROS_OK;
}

int zros_node_add_pub(struct zros_node* node, struct zros_pub* pub)
{
    ZROS_RC(_zros_node_lock(node), return rc);
    sys_slist_append(&node->_pubs, &pub->_list_node);
    _zros_node_unlock(node);
    return ZROS_OK;
}

void zros_node_fini(struct zros_node* node)
{
    zros_broker_remove_node(node);
};

int zros_node_get_name(const struct zros_node* node, char* buf, size_t n)
{
    ZROS_RC(_zros_node_lock((struct zros_node*)node), return rc);
    ZROS_RC(snprintf(buf, n, "%s", node->_name), return rc);
    _zros_node_unlock((struct zros_node*)node);
    return ZROS_OK;
};

// vi: ts=4 sw=4 et
