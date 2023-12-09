/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>
#include <zros/private/zros_broker_struct.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_broker.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>

LOG_MODULE_REGISTER(zros, CONFIG_CEREBRI_SYNAPSE_ZROS_LOG_LEVEL);

static const k_timeout_t g_broker_timeout = K_MSEC(1);
static struct zros_broker _broker = {
    ._nodes = SYS_SLIST_STATIC_INIT(_broker._nodes),
    ._topics = SYS_SLIST_STATIC_INIT(_broker._topics),
    ._lock = Z_MUTEX_INITIALIZER(_broker._lock),
}; // singleton

/********************************************************************
 * zros broker
 ********************************************************************/
int _zros_broker_lock()
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(k_mutex_lock(&_broker._lock, g_broker_timeout),
            LOG_ERR("failed to lock broker");
            return rc);
    return ZROS_OK;
};

void _zros_broker_unlock()
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    k_mutex_unlock(&_broker._lock);
};

int zros_broker_add_node(struct zros_node* node)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    sys_slist_append(&_broker._nodes, &node->_broker_list_node);
    _zros_broker_unlock();
    return ZROS_OK;
}

int zros_broker_remove_node(struct zros_node* node)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    sys_slist_find_and_remove(&_broker._nodes, &node->_broker_list_node);
    _zros_broker_unlock();
    return ZROS_OK;
}

int zros_broker_iterate_nodes(zros_node_iterator_t* iter, void* data)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    struct zros_node* node;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &_broker._nodes, node, _broker_list_node)
    {
        iter(node, data);
    }
    _zros_broker_unlock();
    return ZROS_OK;
}

int zros_broker_add_topic(struct zros_topic* topic)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    sys_slist_append(&_broker._topics, &topic->_broker_list_node);
    _zros_broker_unlock();
    return ZROS_OK;
}

int zros_broker_remove_topic(struct zros_topic* topic)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    sys_slist_find_and_remove(&_broker._topics, &topic->_broker_list_node);
    _zros_broker_unlock();
    return ZROS_OK;
}

int zros_broker_iterate_topic(zros_topic_iterator_t* iter, void* data)
{
    __ASSERT(&_broker != NULL, "zros broker is null");
    ZROS_RC(_zros_broker_lock(), return rc);
    struct zros_topic* topic;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &_broker._topics, topic, _broker_list_node)
    {
        iter(topic, data);
    }
    _zros_broker_unlock();
    return ZROS_OK;
}

// vi: ts=4 sw=4 et
