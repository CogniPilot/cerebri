#ifndef RDD2_IMU_LATENCY_STATS_H_
#define RDD2_IMU_LATENCY_STATS_H_

#include <stdbool.h>
#include <stdint.h>

struct rdd2_imu_latency_stats_snapshot {
	uint32_t last_us;
	uint32_t min_us;
	uint32_t max_us;
	float mean_us;
	float stddev_us;
	uint64_t sample_count;
};

void rdd2_imu_latency_stats_reset(void);
void rdd2_imu_latency_stats_update(uint32_t latency_us);
bool rdd2_imu_latency_stats_get(struct rdd2_imu_latency_stats_snapshot *snapshot);

#endif
