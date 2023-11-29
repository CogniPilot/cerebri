/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/private/zros_topic_struct.h>
#include <zros/zros_common.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>
#include <zros/zros_topic.h>

/********************************************************************
 * zros topic
 ********************************************************************/

LOG_MODULE_DECLARE(zros);

static const k_timeout_t g_topic_timeout = K_MSEC(1);

int _zros_topic_read_write_lock(struct zros_topic* topic)
{
    struct k_sem* read = &topic->_sem_read;
    struct k_mutex* write = &topic->_lock_write;

    // take write semaphore
    ZROS_RC(k_mutex_lock(write, g_topic_timeout),
            LOG_ERR("write lock failed\n");
            return rc);

    // take read semaphore
    unsigned int read_take_count = 0;
    while (true) {
        int rc = k_sem_take(read, g_topic_timeout);
        if (rc != 0) {
            char name[30];
            zros_topic_get_name(topic, name, sizeof(name));
            LOG_ERR("topic %s take read: %u/%u failed\n", name, read_take_count, read->limit);
            for (size_t i = 0; i < read_take_count; i++) {
                k_sem_give(read);
            }
            return rc;
        }
        if (++read_take_count >= read->limit)
            break;
    }
    return ZROS_OK;
}

void _zros_topic_read_write_unlock(struct zros_topic* topic)
{
    struct k_sem* read = &topic->_sem_read;
    struct k_mutex* write = &topic->_lock_write;

    // release read locks
    unsigned int read_take_count = read->limit;
    while (true) {
        k_sem_give(read);
        if (--read_take_count == 0)
            break;
    }

    // release write
    k_mutex_unlock(write);
};

int _zros_topic_read_lock(const struct zros_topic* topic)
{
    struct k_sem* read = (struct k_sem*)&topic->_sem_read;
    ZROS_RC(k_sem_take(read, g_topic_timeout),
            LOG_ERR("take read failed\n");
            return rc);
    return ZROS_OK;
};

void _zros_topic_read_unlock(const struct zros_topic* topic)
{
    struct k_sem* read = (struct k_sem*)&topic->_sem_read;
    k_sem_give(read);
};

int zros_topic_add_pub(struct zros_topic* topic, struct zros_pub* pub)
{
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("pub read lock failed");
            return rc);
    sys_slist_append(&topic->_pubs, &pub->_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_remove_pub(struct zros_topic* topic, struct zros_pub* pub)
{
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("pub read lock failed");
            return rc);
    sys_slist_find_and_remove(&topic->_pubs, &pub->_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_add_sub(struct zros_topic* topic, struct zros_sub* sub)
{
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    sys_slist_append(&topic->_subs, &sub->_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_remove_sub(struct zros_topic* topic, struct zros_sub* sub)
{
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    sys_slist_find_and_remove(&topic->_subs, &sub->_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_publish(struct zros_topic* topic, void* data)
{
    // read/write lock
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic r/w lock failed");
            return rc);

    // write latest data for subscribers
    memcpy(topic->_data, data, topic->_size);

    // write data to subscribers
    struct zros_sub* sub;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &topic->_subs, sub, _list_node)
    {
        int64_t now = k_uptime_ticks();
        double hz = (double)CONFIG_SYS_CLOCK_TICKS_PER_SEC / (now - sub->_last_update_ticks);
        if (hz <= sub->_rate_limit_hz) {
            k_poll_signal_raise(&sub->_data_ready, 1);
            sub->_last_update_ticks = now;
        }
    }

    // read/write unlock
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_read(struct zros_topic* topic, void* data)
{
    ZROS_RC(_zros_topic_read_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    memcpy(data, topic->_data, topic->_size);
    _zros_topic_read_unlock(topic);
    return ZROS_OK;
}

int zros_topic_get_name(const struct zros_topic* topic, char* buf, size_t n)
{
    // name is const char, threadsafe
    ZROS_RC(snprintf(buf, n, "%s", topic->_name), return rc);
    return ZROS_OK;
};

// vi: ts=4 sw=4 et
