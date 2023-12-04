/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_TOPIC_STRUCT_H
#define ZROS_TOPIC_STRUCT_H

#include <zephyr/kernel.h>

/********************************************************************
 * zros topic
 ********************************************************************/
struct zros_topic {
    sys_slist_t _subs; // list of subscriptions
    sys_slist_t _pubs; // list of publications
    void* _data; // data pointer for subscriber pull
    int _size; // size of data
    const char* _name;
    sys_snode_t _list_node; // linked list node
    struct k_sem _sem_read; // read semaphore
    struct k_mutex _lock_write; // write mutex
};

// vi: ts=4 sw=4 et
#endif // ZROS_TOPIC_STRUCT_H
