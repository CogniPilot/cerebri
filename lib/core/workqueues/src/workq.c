/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(core_workqueues, CONFIG_CEREBRI_CORE_WORKQUEUES_LOG_LEVEL);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 6

#define LOW_PRIORITY_STACK_SIZE 8192
#define LOW_PRIORITY_PRIORITY 0

#define HIGH_PRIORITY_STACK_SIZE 16384
#define HIGH_PRIORITY_PRIORITY -1

K_THREAD_STACK_DEFINE(high_priority_stack_area, HIGH_PRIORITY_STACK_SIZE);
K_THREAD_STACK_DEFINE(low_priority_stack_area, LOW_PRIORITY_STACK_SIZE);

struct k_work_q g_high_priority_work_q, g_low_priority_work_q;

int core_workqueues_entry_point(void)
{
    // high priority
    k_work_queue_init(&g_high_priority_work_q);
    struct k_work_queue_config high_priority_cfg = {
        .name = "high_priority_q",
        .no_yield = true
    };
    k_work_queue_start(
        &g_high_priority_work_q,
        high_priority_stack_area,
        K_THREAD_STACK_SIZEOF(high_priority_stack_area),
        HIGH_PRIORITY_PRIORITY,
        &high_priority_cfg);

    // low priority
    k_work_queue_init(&g_low_priority_work_q);
    struct k_work_queue_config low_priority_cfg = {
        .name = "low_priority_q",
        .no_yield = false
    };
    k_work_queue_start(
        &g_low_priority_work_q,
        low_priority_stack_area,
        K_THREAD_STACK_SIZEOF(low_priority_stack_area),
        LOW_PRIORITY_PRIORITY,
        &low_priority_cfg);
    return 0;
}

K_THREAD_DEFINE(core_workqueues, THREAD_STACK_SIZE,
    core_workqueues_entry_point, NULL, NULL, NULL,
    THREAD_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
