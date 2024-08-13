/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_DECLARE(core_common);

sys_slist_t g_perf_counter_list = {
    .head = NULL,
    .tail = NULL
};

struct perf_counter {
    sys_snode_t node;
    const char* name;
    uint32_t deadline_cyc;
    uint32_t min_period_cyc;
    uint32_t max_period_cyc;
    uint64_t misses;
    int64_t last_cyc;
    uint64_t delta_cyc_sum;
    uint64_t count;
};

void perf_counter_init(struct perf_counter* counter, const char* name, double deadline_sec)
{
    counter->name = name;
    counter->deadline_cyc = deadline_sec * sys_clock_hw_cycles_per_sec();
    counter->min_period_cyc = 0;
    counter->max_period_cyc = 0;
    counter->misses = 0;
    counter->last_cyc = 0;
    counter->delta_cyc_sum = 0;
    counter->count = 0;
    sys_slist_append(&g_perf_counter_list, &counter->node);
};

void perf_counter_fini(struct perf_counter* counter)
{
    sys_slist_find_and_remove(&g_perf_counter_list, &counter->node);
};

void perf_counter_update(struct perf_counter* counter)
{
    uint64_t now_cyc = k_cycle_get_64();
    counter->count++;
    if (counter->last_cyc == 0) {
        counter->last_cyc = now_cyc;
    } else {
        int32_t delta_cyc = now_cyc - counter->last_cyc;
        counter->delta_cyc_sum += delta_cyc;
        if (delta_cyc > counter->deadline_cyc) {
            // LOG_WRN("miss detected on: %s, %lld (ns)", counter->name,
            //         1000000000LL*delta_cyc/sys_clock_hw_cycles_per_sec());
            counter->misses++;
        }
        if (counter->min_period_cyc == 0 || delta_cyc < counter->min_period_cyc) {
            counter->min_period_cyc = delta_cyc;
        }
        if (counter->max_period_cyc == 0 || delta_cyc > counter->max_period_cyc) {
            counter->max_period_cyc = delta_cyc;
        }
    }
    counter->last_cyc = now_cyc;
};

int perf_counter_report(struct perf_counter* counter, char* buf, size_t n)
{
    return snprintf(buf, n, "name: %s, max period (ns): %lld, min period (ns): %lld, avg period (ns): %lld, misses: %lld, count: %lld\n",
        counter->name,
        1000000000LL * counter->max_period_cyc / sys_clock_hw_cycles_per_sec(),
        1000000000LL * counter->min_period_cyc / sys_clock_hw_cycles_per_sec(),
        1000000000LL * (counter->delta_cyc_sum / counter->count) / sys_clock_hw_cycles_per_sec(),
        counter->misses, counter->count);
};

void perf_counter_list_report(char* buf, size_t n)
{
    int offset = 0;
    struct perf_counter* counter;
    SYS_SLIST_FOR_EACH_CONTAINER(&g_perf_counter_list, counter, node)
    {
        printf("iterating");
        if (n - offset > 0) {
            offset += perf_counter_report(counter, &buf[offset], n - offset);
        }
    }
};

static char report_buf[1024];

void shell_perf_counter(const struct shell* sh, size_t argc, char** argv, void* data)
{
    perf_counter_list_report(report_buf, ARRAY_SIZE(report_buf));
    shell_print(sh, "%s", report_buf);
}

SHELL_CMD_REGISTER(perf_counter, NULL, "Display perf counters", shell_perf_counter);

// vi: ts=4 sw=4 et
