/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zros/zros_topic.h>

/********************************************************************
 * zros topic
 ********************************************************************/
int zros_topic_read_write_lock(zros_topic_t* topic, k_timeout_t timeout)
{
    int rc = 0;
    struct k_sem* read = &topic->sem_read;
    struct k_mutex* write = &topic->lock_write;

    // take write semaphore
    rc = k_mutex_lock(write, timeout);
    if (rc != 0) {
        printf("write lock failed\n");
        return rc;
    }

    // take read semaphore
    unsigned int read_take_count = 0;
    while (true) {
        rc = k_sem_take(read, timeout);
        if (rc != 0) {
            printf("take read: %d/%d failed\n", read_take_count, read->limit);
            for (size_t i = 0; i < read_take_count; i++) {
                k_sem_give(read);
            }
            return rc;
        }
        if (++read_take_count >= read->limit)
            break;
    }
    return rc;
}

void zros_topic_read_write_unlock(zros_topic_t* topic)
{
    struct k_sem* read = &topic->sem_read;
    struct k_mutex* write = &topic->lock_write;

    // release read locks
    unsigned int read_take_count = read->limit;
    while (true) {
        k_sem_give(read);
        if (--read_take_count <= 0)
            break;
    }

    // release write
    k_mutex_unlock(write);
};

int zros_topic_read_lock(zros_topic_t* topic, k_timeout_t timeout)
{
    int rc = 0;
    struct k_sem* read = &topic->sem_read;
    rc = k_sem_take(read, timeout);
    if (rc != 0) {
        printf("take read failed\n");
        return rc;
    }
    return rc;
};

void zros_topic_read_unlock(zros_topic_t* topic)
{
    struct k_sem* read = &topic->sem_read;
    k_sem_give(read);
};

// vi: ts=4 sw=4 et
