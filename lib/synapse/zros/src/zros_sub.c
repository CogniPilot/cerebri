/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>

#include <zros/private/zros_sub_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

/********************************************************************
 * zros_sub
 ********************************************************************/

LOG_MODULE_DECLARE(zros);

int zros_sub_init(struct zros_sub* sub, struct zros_node* node, struct zros_topic* topic, void* data,
    double rate_limit_hz)
{
    __ASSERT(sub != NULL, "zros sub is null");
    __ASSERT(node != NULL, "zros node is null");
    __ASSERT(topic != NULL, "zros topic is null");
    __ASSERT(data != NULL, "zros data is null");

    // add pub to node
    ZROS_RC(zros_node_add_sub(node, sub),
            LOG_ERR("failed to add push sub to node");
            return rc);

    // set data
    sub->_topic = topic;
    sub->_data = data;
    k_poll_signal_init(&sub->_data_ready);
    sub->_rate_limit_hz = rate_limit_hz;
    sub->_last_update_ticks = 0;
    sub->_node_list_node.next = NULL;
    sub->_topic_list_node.next = NULL;
    k_poll_event_init(&sub->_event, K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY, &sub->_data_ready);
    sub->_node = node;
    return zros_topic_add_sub(topic, sub);
}

int zros_sub_update(struct zros_sub* sub)
{
    __ASSERT(sub != NULL, "zros sub is null");
    return zros_topic_read(sub->_topic, sub->_data);
}

bool zros_sub_update_available(struct zros_sub* sub)
{
    __ASSERT(sub != NULL, "zros sub is null");
    if (sub->_event.signal->signaled == 1) {
        sub->_event.signal->signaled = 0;
        sub->_event.state = K_POLL_STATE_NOT_READY;
        return true;
    } else {
        return false;
    }
}

struct k_poll_event* zros_sub_get_event(struct zros_sub* sub)
{
    __ASSERT(sub != NULL, "zros sub is null");
    return &sub->_event;
}

void zros_sub_fini(struct zros_sub* sub)
{
    __ASSERT(sub != NULL, "zros sub is null");
    zros_topic_remove_sub(sub->_topic, sub);
}

void zros_sub_get_node(struct zros_sub* sub, struct zros_node** node)
{
    __ASSERT(sub != NULL, "zros sub is null");
    node = &sub->_node;
};

// vi: ts=4 sw=4 et
