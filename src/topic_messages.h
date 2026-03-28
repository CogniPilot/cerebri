#ifndef CEREBRI2_TOPIC_MESSAGES_H_
#define CEREBRI2_TOPIC_MESSAGES_H_

#include <stdbool.h>
#include <stdint.h>

struct rc_frame {
	int32_t us[16];
	int64_t stamp_ms;
	bool valid;
};

struct imu_sample {
	float gyro_rad_s[3];
	float accel_m_s2[3];
};

struct rate_triplet {
	float roll;
	float pitch;
	float yaw;
};

struct rc_channels_msg {
	int32_t us[16];
};

struct control_status_msg {
	int64_t rc_stamp_ms;
	int32_t throttle_us;
	uint8_t rc_link_quality;
	bool armed;
	bool rc_valid;
	bool rc_stale;
	bool imu_ok;
	bool arm_switch;
};

struct rate_debug_msg {
	struct rate_triplet desired;
	struct rate_triplet cmd;
};

struct flight_snapshot {
	struct imu_sample imu;
	struct rc_channels_msg rc;
	struct control_status_msg status;
	struct rate_debug_msg rate;
};

struct motor_output_msg {
	float motors[4];
	uint16_t raw[4];
	bool armed;
	bool test_mode;
};

#endif
