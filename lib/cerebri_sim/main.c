/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "clock.pb.h"
#include <zephyr/kernel.h>

#include <stdio.h>
#include <synapse_zbus/channels.h>

#include <pthread.h>
#include <sys/socket.h>

#define MY_STACK_SIZE 500
#define MY_PRIORITY -10

int64_t connect_time = 0;
Clock sim_clock = Clock_init_default;

#define PUB_SIM_MESSAGES(TOPIC, MSG)                        \
    {                                                       \
        msg_##MSG##_t data;                                 \
        while (queue_##TOPIC.tryPop(data)) {                \
            if (data.timestamp == 0) {                      \
                data.timestamp = uptime;                    \
            }                                               \
            zbus_chan_pub(&chan_##TOPIC, &data, K_FOREVER); \
        }                                                   \
    }

void sim_clock_callback(const struct zbus_channel* chan)
{
    sim_clock = *(const Clock*)zbus_chan_const_msg(chan);
    // printf("sim clock callback\n");
}

void thread_sim_core_entry_point(void* p1, void* p2, void* p3)
{
    printf("sim core running\n");
    while (true) {
        int64_t uptime = k_uptime_get();
        int64_t sec = uptime / 1.0e3;
        int32_t nsec = (uptime - sec * 1e3) * 1e6;

        // fast forward zephyClock time to match sim
        int64_t delta_sec = sim_clock.sim.sec - sec;
        int32_t delta_nsec = sim_clock.sim.nsec - nsec;
        int32_t wait = delta_sec * 1e6 + delta_nsec / 1e3;

        printf("sim: sec %ld nsec %d\n", sim_clock.sim.sec, sim_clock.sim.nsec);
        printf("uptime: sec %ld nsec %d\n", sec, nsec);
        printf("wait millis: %d\n", wait);
        if (wait > 0) {
            k_usleep(wait);
        } else {
            k_usleep(1);
        }
    }
}

K_THREAD_DEFINE(sim_core, MY_STACK_SIZE, thread_sim_core_entry_point,
    NULL, NULL, NULL, MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
