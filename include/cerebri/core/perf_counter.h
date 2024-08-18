#ifndef CEREBRI_CORE_PERF_COUNTER_H
#define CEREBRI_CORE_PERF_COUNTER_H

#include <zephyr/kernel.h>

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

void perf_counter_init(struct perf_counter* counter, const char* name, double max_period_sec);

void perf_counter_fini(struct perf_counter* counter);

void perf_counter_update(struct perf_counter* counter);

int perf_counter_report(struct perf_counter* counter, char* buf, size_t n);

void perf_counter_list_report(char* buf, size_t n);

// vi: ts=4 sw=4 et

#endif // CEREBRI_CORE_PERF_COUNTER_H
