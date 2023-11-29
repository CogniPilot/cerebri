/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>

#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_common.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

/********************************************************************
 * zros pub
 ********************************************************************/

LOG_MODULE_DECLARE(zros);

int zros_pub_init(struct zros_pub* pub, struct zros_node* node, struct zros_topic* topic, void* data)
{
    // add pub to node
    ZROS_RC(zros_node_add_pub(node, pub),
            LOG_ERR("failed to add pub to node\n");
            return rc);

    // set data
    pub->_list_node.next = NULL;
    pub->_data = data;
    pub->_topic = topic;

    return zros_topic_add_pub(topic, pub);
};

int zros_pub_update(struct zros_pub* pub)
{
    return zros_topic_publish(pub->_topic, pub->_data);
}

void zros_pub_fini(struct zros_pub* pub)
{
    zros_topic_remove_pub(pub->_topic, pub);
};

// vi: ts=4 sw=4 et
