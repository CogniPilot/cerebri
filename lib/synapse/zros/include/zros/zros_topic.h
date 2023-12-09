/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_TOPIC_H
#define ZROS_TOPIC_H
#include <zephyr/kernel.h>

/********************************************************************
 * zros topic
 ********************************************************************/

#define ZROS_TOPIC_DEFINE(NAME, TYPE)                                 \
    static TYPE g_msg_##NAME = {};                                    \
    struct zros_topic topic_##NAME = {                                \
        ._name = #NAME,                                               \
        ._data = &g_msg_##NAME,                                       \
        ._size = sizeof(g_msg_##NAME),                                \
        ._subs = SYS_SLIST_STATIC_INIT(topic_##NAME._subs),           \
        ._pubs = SYS_SLIST_STATIC_INIT(topic_##NAME._subs),           \
        ._broker_list_node = {                                        \
            .next = NULL,                                             \
        },                                                            \
        ._sem_read = Z_SEM_INITIALIZER(topic_##NAME._sem_read, 6, 6), \
        ._lock_write = Z_MUTEX_INITIALIZER(topic_##NAME._lock_write), \
    };

#define ZROS_TOPIC_DECLARE(NAME, TYPE) \
    extern struct zros_topic NAME;

// forward declarations
struct zros_topic;
struct zros_sub;
struct zros_pub;

// public api
typedef void zros_pub_iterator_t(const struct zros_pub* pub, void* data);
typedef void zros_sub_iterator_t(const struct zros_sub* sub, void* data);
int zros_topic_publish(struct zros_topic* topic, void* data);
int zros_topic_read(struct zros_topic* topic, void* data);
int zros_topic_get_name(const struct zros_topic* node, char* buf, size_t n);
int zros_topic_add_pub(struct zros_topic* topic, struct zros_pub* pub);
int zros_topic_remove_pub(struct zros_topic* topic, struct zros_pub* pub);
int zros_topic_add_sub(struct zros_topic* topic, struct zros_sub* sub);
int zros_topic_remove_sub(struct zros_topic* topic, struct zros_sub* sub);
int zros_topic_iterate_pub(struct zros_topic* topic, zros_pub_iterator_t* iter, void* data);
int zros_topic_iterate_sub(struct zros_topic* topic, zros_sub_iterator_t* iter, void* data);

// vi: ts=4 sw=4 et
#endif // ZROS_TOPIC_H
