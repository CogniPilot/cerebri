/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <zros/zros_pub.h>
#include <zros/zros_push_sub.h>

/********************************************************************
 * zros pub
 ********************************************************************/
int zros_pub_init(zros_pub_t* pub, zros_topic_t* topic, void* data, k_timeout_t timeout)
{
    int rc = 0;

    // set data pointer
    pub->data = data;

    // obtain read/write lock
    rc = zros_topic_read_write_lock(topic, timeout);
    if (rc != 0) {
        printf("read write lock failed on pub init\n");
        return rc;
    }

    // set topic
    pub->topic = topic;

    // add publisher to topic
    sys_slist_append(&topic->pubs, &pub->list_node);

    // release read/write lock
    zros_topic_read_write_unlock(topic);
    return rc;
};

int zros_pub_update(zros_pub_t* pub, k_timeout_t timeout)
{
    int rc = 0;
    // lock
    rc = zros_topic_read_write_lock(pub->topic, timeout);
    if (rc != 0) {
        printf("read write lock failed on pub push\n");
        return rc;
    }

    // write data
    memcpy(pub->topic->data, pub->data, pub->topic->size);

    // write data to push subscribers
    zros_push_sub_t* sub;
    SYS_SLIST_FOR_EACH_CONTAINER(&pub->topic->push_subs, sub, list_node)
    {
        int64_t now = k_uptime_ticks();
        double hz = (double)CONFIG_SYS_CLOCK_TICKS_PER_SEC / (now - sub->last_update_ticks);
        if (hz <= sub->rate_limit_hz) {
            k_poll_signal_raise(&sub->data_ready, 1);
            sub->last_update_ticks = now;
        }
    }

    // unlock
    zros_topic_read_write_unlock(pub->topic);
    return rc;
}

// vi: ts=4 sw=4 et
