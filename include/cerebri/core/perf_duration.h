#ifndef CEREBRI_CORE_PERF_DURATION_H
#define CEREBRI_CORE_PERF_DURATION_H

#include <zephyr/kernel.h>

struct perf_duration {
	sys_snode_t node;
	bool started;
	const char *name;
	uint32_t deadline_cyc;
	uint32_t min_duration_cyc;
	uint32_t max_duration_cyc;
	uint64_t misses;
	int64_t start_cyc;
	uint64_t delta_cyc_sum;
	uint64_t count;
};

void perf_duration_init(struct perf_duration *duration, const char *name, double max_period_sec);

void perf_duration_fini(struct perf_duration *duration);

void perf_duration_start(struct perf_duration *duration);

void perf_duration_stop(struct perf_duration *duration);

int perf_duration_report(struct perf_duration *duration, char *buf, size_t n);

void perf_duration_list_report(char *buf, size_t n);

// vi: ts=4 sw=4 et

#endif // CEREBRI_CORE_PERF_duration_H
