/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SYNAPSE_ZBUS_SYN_PUB_SUB_H
#define SYNAPSE_ZBUS_SYN_PUB_SUB_H

#include <zephyr/kernel.h>

#include <synapse/zbus/common.h>

struct syn_sub {
    const struct zbus_channel* chan;
    void* msg;
    struct k_poll_signal signal;
    struct k_poll_event events[1];
    struct k_mutex mutex;
};

int syn_sub_init(
    struct syn_sub* sub,
    void* msg,
    const struct zbus_channel* chan);

int syn_sub_poll(struct syn_sub* sub, k_timeout_t timeout);

int syn_sub_listen(struct syn_sub* sub, const struct zbus_channel* chan, k_timeout_t timeout);

int syn_sub_claim(struct syn_sub* sub, k_timeout_t timeout);

int syn_sub_finish(struct syn_sub* sub);

struct syn_pub {
    const struct zbus_channel* chan;
    void* msg;
    struct k_mutex mutex;
};

int syn_pub_init(
    struct syn_pub* pub,
    void* msg,
    const struct zbus_channel* chan);

int syn_pub_claim(struct syn_pub* pub, k_timeout_t timeout);

int syn_pub_finish(struct syn_pub* pub);

int syn_pub_publish(struct syn_pub* pub, k_timeout_t timeout);

struct syn_sub_nocopy {
    const struct zbus_channel* chan;
    struct k_poll_signal signal;
    struct k_poll_event events[1];
};

int syn_sub_nocopy_init(
    struct syn_sub_nocopy* sub,
    const struct zbus_channel* chan);

int syn_sub_nocopy_poll(struct syn_sub_nocopy* sub, k_timeout_t timeout);

int syn_sub_nocopy_listen(struct syn_sub_nocopy* sub, k_timeout_t timeout);

int syn_sub_nocopy_claim(struct syn_sub_nocopy* sub, k_timeout_t timeout);

int syn_sub_nocopy_finish(struct syn_sub_nocopy* sub);

#endif // SYNAPSE_ZBUS_SYN_PUB_SUB_H
/* vi: ts=4 sw=4 et */
