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
#include <zros/zros_node.h>
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
    __ASSERT(topic != NULL, "zros topic is null");
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
            char name[20];
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
    __ASSERT(topic != NULL, "zros topic is null");
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
    __ASSERT(topic != NULL, "zros topic is null");
    struct k_sem* read = (struct k_sem*)&topic->_sem_read;
    ZROS_RC(k_sem_take(read, g_topic_timeout),
            LOG_ERR("take read failed\n");
            return rc);
    return ZROS_OK;
};

void _zros_topic_read_unlock(const struct zros_topic* topic)
{
    __ASSERT(topic != NULL, "zros topic is null");
    struct k_sem* read = (struct k_sem*)&topic->_sem_read;
    k_sem_give(read);
};

int zros_topic_add_pub(struct zros_topic* topic, struct zros_pub* pub)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("pub read lock failed");
            return rc);
    sys_slist_append(&topic->_pubs, &pub->_topic_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_remove_pub(struct zros_topic* topic, struct zros_pub* pub)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("pub read lock failed");
            return rc);
    sys_slist_find_and_remove(&topic->_pubs, &pub->_topic_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_add_sub(struct zros_topic* topic, struct zros_sub* sub)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    sys_slist_append(&topic->_subs, &sub->_topic_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_remove_sub(struct zros_topic* topic, struct zros_sub* sub)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    sys_slist_find_and_remove(&topic->_subs, &sub->_topic_list_node);
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_publish(struct zros_topic* topic, void* data)
{
    __ASSERT(topic != NULL, "zros topic is null");
    char topic_name[30];
    char node_name[30];

    // read/write lock
    ZROS_RC(_zros_topic_read_write_lock(topic),
            LOG_ERR("topic r/w lock failed");
            return rc);

    zros_topic_get_name(topic, topic_name, sizeof(topic_name));

    // write latest data for subscribers
    memcpy(topic->_data, data, topic->_size);
    // LOG_WRN("\npublishing topic %s", topic_name);

    // write data to subscribers
    struct zros_sub* sub;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &topic->_subs, sub, _topic_list_node)
    {
        int64_t now = k_uptime_ticks();
        zros_node_get_name(sub->_node, node_name, sizeof(node_name));
        double hz = (double)CONFIG_SYS_CLOCK_TICKS_PER_SEC / (now - sub->_last_update_ticks);
        if (hz <= sub->_rate_limit_hz) {
            k_poll_signal_raise(&sub->_data_ready, 1);
            sub->_last_update_ticks = now;
            // LOG_WRN("subscriber %s: update", node_name);
        } else {
            // LOG_WRN("subscriber %s: ignore", node_name);
        }
    }

    // read/write unlock
    _zros_topic_read_write_unlock(topic);
    return ZROS_OK;
}

int zros_topic_read(struct zros_topic* topic, void* data)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    memcpy(data, topic->_data, topic->_size);
    _zros_topic_read_unlock(topic);
    return ZROS_OK;
}

int zros_topic_get_name(const struct zros_topic* topic, char* buf, size_t n)
{
    __ASSERT(topic != NULL, "zros topic is null");
    // name is const char, threadsafe
    ZROS_RC(snprintf(buf, n, "%s", topic->_name), return rc);
    return ZROS_OK;
};

int zros_topic_iterate_pub(struct zros_topic* topic, zros_pub_iterator_t* iter, void* data)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    struct zros_pub* pub;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &topic->_pubs, pub, _topic_list_node)
    {
        __ASSERT(pub != NULL, "pub is null");
        iter(pub, data);
    }
    _zros_topic_read_unlock(topic);
    return ZROS_OK;
}

int zros_topic_iterate_sub(struct zros_topic* topic, zros_sub_iterator_t* iter, void* data)
{
    __ASSERT(topic != NULL, "zros topic is null");
    ZROS_RC(_zros_topic_read_lock(topic),
            LOG_ERR("topic read lock failed");
            return rc);
    struct zros_sub* sub;
    SYS_SLIST_FOR_EACH_CONTAINER(
        &topic->_subs, sub, _topic_list_node)
    {
        __ASSERT(sub != NULL, "sub is null");
        iter(sub, data);
    }
    _zros_topic_read_unlock(topic);
    return ZROS_OK;
}

// vi: ts=4 sw=4 et
