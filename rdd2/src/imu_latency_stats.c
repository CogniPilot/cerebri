/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_latency_stats.h"
#include "hotpath_memory.h"

#include <math.h>
#include <string.h>

struct rdd2_imu_latency_stats_state {
	uint32_t sequence;
	struct rdd2_imu_latency_stats_snapshot snapshot;
	uint32_t last_us;
	uint32_t min_us;
	uint32_t max_us;
	uint64_t sample_count;
	double mean_us;
	double m2_us2;
};

static RDD2_HOTPATH_DTCM_BSS struct rdd2_imu_latency_stats_state g_rdd2_imu_latency_stats;

static void rdd2_imu_latency_stats_publish(const struct rdd2_imu_latency_stats_snapshot *snapshot)
{
	__atomic_add_fetch(&g_rdd2_imu_latency_stats.sequence, 1U, __ATOMIC_RELAXED);
	__atomic_thread_fence(__ATOMIC_RELEASE);
	g_rdd2_imu_latency_stats.snapshot = *snapshot;
	__atomic_thread_fence(__ATOMIC_RELEASE);
	__atomic_add_fetch(&g_rdd2_imu_latency_stats.sequence, 1U, __ATOMIC_RELAXED);
}

void rdd2_imu_latency_stats_reset(void)
{
	memset(&g_rdd2_imu_latency_stats, 0, sizeof(g_rdd2_imu_latency_stats));
}

void rdd2_imu_latency_stats_update(uint32_t latency_us)
{
	struct rdd2_imu_latency_stats_snapshot snapshot;
	double delta;
	double delta2;
	double variance_us2;

	if (latency_us == 0U) {
		return;
	}

	g_rdd2_imu_latency_stats.sample_count++;
	g_rdd2_imu_latency_stats.last_us = latency_us;

	if (g_rdd2_imu_latency_stats.sample_count == 1U) {
		g_rdd2_imu_latency_stats.min_us = latency_us;
		g_rdd2_imu_latency_stats.max_us = latency_us;
		g_rdd2_imu_latency_stats.mean_us = (double)latency_us;
		g_rdd2_imu_latency_stats.m2_us2 = 0.0;
	} else {
		if (latency_us < g_rdd2_imu_latency_stats.min_us) {
			g_rdd2_imu_latency_stats.min_us = latency_us;
		}
		if (latency_us > g_rdd2_imu_latency_stats.max_us) {
			g_rdd2_imu_latency_stats.max_us = latency_us;
		}

		delta = (double)latency_us - g_rdd2_imu_latency_stats.mean_us;
		g_rdd2_imu_latency_stats.mean_us +=
			delta / (double)g_rdd2_imu_latency_stats.sample_count;
		delta2 = (double)latency_us - g_rdd2_imu_latency_stats.mean_us;
		g_rdd2_imu_latency_stats.m2_us2 += delta * delta2;
	}

	variance_us2 = 0.0;
	if (g_rdd2_imu_latency_stats.sample_count > 1U) {
		variance_us2 = g_rdd2_imu_latency_stats.m2_us2 /
			       (double)g_rdd2_imu_latency_stats.sample_count;
	}

	snapshot = (struct rdd2_imu_latency_stats_snapshot){
		.last_us = g_rdd2_imu_latency_stats.last_us,
		.min_us = g_rdd2_imu_latency_stats.min_us,
		.max_us = g_rdd2_imu_latency_stats.max_us,
		.mean_us = (float)g_rdd2_imu_latency_stats.mean_us,
		.stddev_us = (float)sqrt(variance_us2),
		.sample_count = g_rdd2_imu_latency_stats.sample_count,
	};
	rdd2_imu_latency_stats_publish(&snapshot);
}

bool rdd2_imu_latency_stats_get(struct rdd2_imu_latency_stats_snapshot *snapshot)
{
	uint32_t start_sequence;
	uint32_t end_sequence;

	if (snapshot == NULL) {
		return false;
	}

	do {
		start_sequence =
			__atomic_load_n(&g_rdd2_imu_latency_stats.sequence, __ATOMIC_ACQUIRE);
		if ((start_sequence & 1U) != 0U) {
			continue;
		}

		*snapshot = g_rdd2_imu_latency_stats.snapshot;
		__atomic_thread_fence(__ATOMIC_ACQUIRE);
		end_sequence =
			__atomic_load_n(&g_rdd2_imu_latency_stats.sequence, __ATOMIC_RELAXED);
	} while (start_sequence != end_sequence || (end_sequence & 1U) != 0U);

	return snapshot->sample_count != 0U;
}
