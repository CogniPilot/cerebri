/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_ZBUS_SYN_PUB_SUB_H
#define SYNAPSE_ZBUS_SYN_PUB_SUB_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define RC(X, Y)       \
    {                  \
        int rc = X;    \
        if (rc != 0) { \
            Y;         \
        }              \
    };

typedef struct syn_sub_s {
    const struct zbus_channel* chan;
    void* msg;
    struct k_poll_signal signal;
    struct k_poll_event events[1];
    struct k_mutex mutex;
    struct syn_sub_s* next;
    int64_t ticks_last;
    int throttle_hz;
} syn_sub_t;

int syn_sub_init(
    syn_sub_t* sub,
    void* msg,
    const struct zbus_channel* chan, int throttle_hz);

int syn_sub_poll(syn_sub_t* sub, k_timeout_t timeout);

int syn_sub_listen(syn_sub_t* sub, const struct zbus_channel* chan, k_timeout_t timeout);

int syn_sub_lock(syn_sub_t* sub, k_timeout_t timeout);

int syn_sub_unlock(syn_sub_t* sub);

typedef struct syn_pub_s {
    const struct zbus_channel* chan;
    void* msg;
    struct k_mutex mutex;
    struct syn_pub_s* next;
} syn_pub_t;

int syn_pub_init(
    syn_pub_t* pub,
    void* msg,
    const struct zbus_channel* chan);

int syn_pub_lock(syn_pub_t* pub, k_timeout_t timeout);

int syn_pub_unlock(syn_pub_t* pub);

int syn_pub_publish(syn_pub_t* pub, k_timeout_t timeout);

int syn_pub_forward(syn_pub_t* pub, const struct zbus_channel* chan, const struct zbus_channel* chan_src, k_timeout_t timeout);

typedef struct syn_node_s {
    const char* name;
    syn_sub_t* sub_list_head;
    syn_pub_t* pub_list_head;
} syn_node_t;

void syn_node_init(syn_node_t* node, const char* name);

int syn_node_add_sub(syn_node_t* node,
    syn_sub_t* sub,
    void* msg,
    const struct zbus_channel* chan,
    int throttle_hz
    );

int syn_node_add_pub(syn_node_t* node,
    syn_pub_t* pub,
    void* msg,
    const struct zbus_channel* chan);

void syn_node_listen(syn_node_t* node,
    const struct zbus_channel* chan, k_timeout_t timeout);

void syn_node_unlock_all(syn_node_t* sub);

void syn_node_lock_all(syn_node_t* sub, k_timeout_t timeout);

void syn_node_publish_all(syn_node_t* node, k_timeout_t timeout);

#endif // SYNAPSE_ZBUS_SYN_PUB_SUB_H
/* vi: ts=4 sw=4 et */
