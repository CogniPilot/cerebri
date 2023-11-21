/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_TOPIC_H_
#define ZROS_TOPIC_H_

#include <zephyr/kernel.h>

/********************************************************************
 * zros topic
 ********************************************************************/
typedef struct zros_topic_s {
    sys_slist_t push_subs; // list of push subscriptions
    sys_slist_t pull_subs; // list of pull subscriptions
    sys_slist_t pubs; // list of publications
    void* data; // data pointer for subscriber pull
    int size; // size of data
    struct k_sem sem_read; // read semaphore
    struct k_mutex lock_write; // write mutex
} zros_topic_t;
int zros_topic_read_write_lock(zros_topic_t* topic, k_timeout_t timeout);
void zros_topic_read_write_unlock(zros_topic_t* topic);
int zros_topic_read_lock(zros_topic_t* topic, k_timeout_t timeout);
void zros_topic_read_unlock(zros_topic_t* topic);

// vi: ts=4 sw=4 et
#endif // ZROS_TOPIC_H_
