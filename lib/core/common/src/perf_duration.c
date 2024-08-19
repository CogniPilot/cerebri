/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cerebri/core/perf_duration.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_DECLARE(core_common);

sys_slist_t g_perf_duration_list = {
    .head = NULL,
    .tail = NULL
};

void perf_duration_init(struct perf_duration* duration, const char* name, double deadline_sec)
{
    duration->started = false;
    duration->name = name;
    duration->deadline_cyc = deadline_sec * sys_clock_hw_cycles_per_sec();
    duration->min_duration_cyc = 0;
    duration->max_duration_cyc = 0;
    duration->misses = 0;
    duration->start_cyc = 0;
    duration->delta_cyc_sum = 0;
    duration->count = 0;
    sys_slist_append(&g_perf_duration_list, &duration->node);
};

void perf_duration_fini(struct perf_duration* duration)
{
    sys_slist_find_and_remove(&g_perf_duration_list, &duration->node);
};

void perf_duration_start(struct perf_duration* duration)
{
    if (!duration->started) {
        duration->start_cyc = k_cycle_get_64();
        duration->started = true;
    }
};

void perf_duration_stop(struct perf_duration* duration)
{
    if (!duration->started) {
        return;
    }
    duration->started = false;
    uint64_t now_cyc = k_cycle_get_64();
    duration->count++;
    int32_t delta_cyc = now_cyc - duration->start_cyc;
    duration->delta_cyc_sum += delta_cyc;
    if (delta_cyc > duration->deadline_cyc) {
        // LOG_WRN("miss detected on: %s, %lld (ns)", duration->name,
        //         1000000000LL*delta_cyc/sys_clock_hw_cycles_per_sec());
        duration->misses++;
    }
    if (duration->min_duration_cyc == 0 || delta_cyc < duration->min_duration_cyc) {
        duration->min_duration_cyc = delta_cyc;
    }
    if (duration->max_duration_cyc == 0 || delta_cyc > duration->max_duration_cyc) {
        duration->max_duration_cyc = delta_cyc;
    }
};

int perf_duration_report(struct perf_duration* duration, char* buf, size_t n)
{
    return snprintf(buf, n, "name: %s, max duration (ns): %lld, min duration (ns): %lld, avg duration (ns): %lld, misses: %lld, count: %lld\n",
        duration->name,
        1000000000LL * duration->max_duration_cyc / sys_clock_hw_cycles_per_sec(),
        1000000000LL * duration->min_duration_cyc / sys_clock_hw_cycles_per_sec(),
        1000000000LL * (duration->delta_cyc_sum / duration->count) / sys_clock_hw_cycles_per_sec(),
        duration->misses, duration->count);
};

void perf_duration_list_report(char* buf, size_t n)
{
    int offset = 0;
    struct perf_duration* duration;
    SYS_SLIST_FOR_EACH_CONTAINER(&g_perf_duration_list, duration, node)
    {
        printf("iterating");
        if (n - offset > 0) {
            offset += perf_duration_report(duration, &buf[offset], n - offset);
        }
    }
};

static char report_buf[1024];

void shell_perf_duration(const struct shell* sh, size_t argc, char** argv, void* data)
{
    perf_duration_list_report(report_buf, ARRAY_SIZE(report_buf));
    shell_print(sh, "%s", report_buf);
}

SHELL_CMD_REGISTER(perf_duration, NULL, "Display perf durations", shell_perf_duration);

struct perf_duration control_latency;

static int perf_duration_sys_init(void)
{
    perf_duration_init(&control_latency, "control_latency", 0.001);
    return 0;
};

SYS_INIT(perf_duration_sys_init, POST_KERNEL, 1);

// vi: ts=4 sw=4 et
