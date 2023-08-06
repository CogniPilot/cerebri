/*
 * Copyright (c) 2022 Rodrigo Peixoto <rodrigopex@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>

#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

LOG_MODULE_REGISTER(zbus_topic_list, LOG_LEVEL_INF);
static int count = 0;

#if defined(CONFIG_ZBUS_STRUCTS_ITERABLE_ACCESS)
static bool print_channel_data_iterator(const struct zbus_channel* chan)
{
    LOG_INF("%d - Channel %s:", count, zbus_chan_name(chan));
    LOG_INF("      Message size: %d", zbus_chan_msg_size(chan));
    ++count;
    LOG_INF("      Observers:");
    for (const struct zbus_observer* const* obs = chan->observers; *obs != NULL; ++obs) {
        LOG_INF("      - %s", (*obs)->name);
    }

    return true;
}

static bool print_observer_data_iterator(const struct zbus_observer* obs)
{
    LOG_INF("%d - %s %s", count, obs->queue ? "Subscriber" : "Listener", zbus_obs_name(obs));

    ++count;

    return true;
}
#endif // CONFIG_ZBUS_STRUCTS_ITERABLE_ACCESS

static int zbus_topic_list(const struct shell* shell,
    size_t argc, char** argv, void* data)
{
    count = 0;

#if defined(CONFIG_ZBUS_STRUCTS_ITERABLE_ACCESS)
    LOG_INF("Channel list:");
    zbus_iterate_over_channels(print_channel_data_iterator);

    count = 0;

    LOG_INF("Observers list:");
    zbus_iterate_over_observers(print_observer_data_iterator);
#endif // CONFIG_ZBUS_STRUCTS_ITERABLE_ACCESS
    return 0;
}

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_zbus, SHELL_CMD(topic, NULL, "List topics.", zbus_topic_list),
    SHELL_SUBCMD_SET_END);
/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(zbus, &sub_zbus, "zbus command", NULL);
