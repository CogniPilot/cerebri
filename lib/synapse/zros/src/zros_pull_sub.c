/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <zros/zros_pull_sub.h>

/********************************************************************
 * zros_pull_sub
 ********************************************************************/
int zros_pull_sub_init(zros_pull_sub_t* sub, zros_topic_t* topic, void* data,
    k_timeout_t timeout)
{
    int rc = 0;

    // set data
    sub->data = data;
    sub->topic = topic;

    // lock
    rc = zros_topic_read_write_lock(topic, timeout);
    if (rc != 0) {
        printf("read write lock failed on sub init\n");
        return rc;
    }

    // add publisher to topic
    sys_slist_append(&topic->pull_subs, &sub->list_node);

    // unlock
    zros_topic_read_write_unlock(topic);
    return rc;
}

int zros_pull_sub_update(zros_pull_sub_t* sub, k_timeout_t timeout)
{
    int rc = 0;
    // lock
    rc = zros_topic_read_lock(sub->topic, timeout);
    if (rc != 0) {
        printf("read lock failed, sub pull\n");
        return rc;
    }

    // read data
    memcpy(sub->data, sub->topic->data, sub->topic->size);

    // unlock
    zros_topic_read_unlock(sub->topic);
    return rc;
}

// vi: ts=4 sw=4 et
