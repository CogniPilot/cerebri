/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>

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
    // add pub to node
    ZROS_RC(zros_node_add_sub(node, sub),
            LOG_ERR("failed to add push sub to node");
            return rc);

    // set data
    sub->_data = data;
    sub->_topic = topic;
    sub->_list_node.next = NULL;
    sub->_rate_limit_hz = rate_limit_hz;
    k_poll_signal_init(&sub->_data_ready);
    k_poll_event_init(&sub->_event, K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY, &sub->_data_ready);
    return zros_topic_add_sub(topic, sub);
}

int zros_sub_update(struct zros_sub* sub)
{
    return zros_topic_read(sub->_topic, sub->_data);
}

bool zros_sub_update_available(struct zros_sub* sub)
{
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
    return &sub->_event;
}

void zros_sub_fini(struct zros_sub* sub)
{
    zros_topic_remove_sub(sub->_topic, sub);
}

// vi: ts=4 sw=4 et
