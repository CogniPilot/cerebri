/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_shell.h"
#include "flight_mode.h"
#include "topic_flatbuffer.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#define TOPIC_THREAD_STACK_SIZE 1536
#define THROTTLE_CHANNEL_INDEX  2
#define TOPIC_VEC3_FMT          "[%9.5f %9.5f %9.5f]"
#define TOPIC_RATE_FMT          "[%9.6f %9.6f %9.6f]"

enum topic_id {
	TOPIC_STATUS = 0,
	TOPIC_IMU,
	TOPIC_RC,
	TOPIC_MOTOR,
	TOPIC_ATTITUDE,
	TOPIC_ATTITUDE_DESIRED,
	TOPIC_DESIRED_ANGULAR_RATE,
	TOPIC_ANGULAR_RATE_CMD,
	TOPIC_COUNT,
	TOPIC_INVALID = -1,
};

enum topic_watch_mode {
	TOPIC_WATCH_NONE = 0,
	TOPIC_WATCH_ECHO,
	TOPIC_WATCH_HZ,
};

struct topic_watch {
	const struct shell *sh;
	enum topic_id topic;
	enum topic_watch_mode mode;
	uint32_t period_ms;
	uint32_t last_seq;
	int64_t last_ms;
	bool active;
};

static struct topic_watch g_topic_watch;
static atomic_t g_topic_seq[TOPIC_COUNT];
static const char *const g_topic_names[] = {
	"status",
	"imu",
	"rc",
	"motor",
	"attitude",
	"attitude_desired",
	"rate_desired",
	"rate_cmd",
};

struct flight_state_store {
	uint8_t slots[2][RDD2_TOPIC_FB_FLIGHT_STATE_SIZE];
	uint16_t lengths[2];
	atomic_t generation;
};

struct motor_output_store {
	uint8_t slots[2][RDD2_TOPIC_FB_MOTOR_OUTPUT_SIZE];
	uint16_t lengths[2];
	atomic_t generation;
};

static struct flight_state_store g_flight_state_store;
static struct motor_output_store g_motor_output_store;

K_SEM_DEFINE(g_topic_watch_sem, 0, 1);

static void topic_watch_thread(void *p0, void *p1, void *p2);
static void topic_dynamic_get(size_t idx, struct shell_static_entry *entry);

K_THREAD_DEFINE(g_topic_watch_tid, TOPIC_THREAD_STACK_SIZE, topic_watch_thread, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
SHELL_DYNAMIC_CMD_CREATE(sub_topic_names, topic_dynamic_get);

static const char *topic_name(enum topic_id topic)
{
	switch (topic) {
	case TOPIC_STATUS:
		return "status";
	case TOPIC_IMU:
		return "imu";
	case TOPIC_RC:
		return "rc";
	case TOPIC_MOTOR:
		return "motor";
	case TOPIC_ATTITUDE:
		return "attitude";
	case TOPIC_ATTITUDE_DESIRED:
		return "attitude_desired";
	case TOPIC_DESIRED_ANGULAR_RATE:
		return "rate_desired";
	case TOPIC_ANGULAR_RATE_CMD:
		return "rate_cmd";
	default:
		return "unknown";
	}
}

static enum topic_id topic_parse(const char *name)
{
	if (strcmp(name, "status") == 0 || strcmp(name, "fc") == 0 ||
	    strcmp(name, "fc_status") == 0) {
		return TOPIC_STATUS;
	}

	if (strcmp(name, "imu") == 0) {
		return TOPIC_IMU;
	}

	if (strcmp(name, "rc") == 0 || strcmp(name, "crsf") == 0) {
		return TOPIC_RC;
	}

	if (strcmp(name, "motor") == 0 || strcmp(name, "motors") == 0 ||
	    strcmp(name, "dshot") == 0) {
		return TOPIC_MOTOR;
	}

	if (strcmp(name, "attitude") == 0 || strcmp(name, "euler") == 0) {
		return TOPIC_ATTITUDE;
	}

	if (strcmp(name, "attitude_desired") == 0 || strcmp(name, "attitude_sp") == 0 ||
	    strcmp(name, "level_sp") == 0) {
		return TOPIC_ATTITUDE_DESIRED;
	}

	if (strcmp(name, "rate_desired") == 0 || strcmp(name, "desired_angular_rate") == 0 ||
	    strcmp(name, "desired_rate") == 0 || strcmp(name, "desired_roll_rate") == 0 ||
	    strcmp(name, "roll_sp") == 0) {
		return TOPIC_DESIRED_ANGULAR_RATE;
	}

	if (strcmp(name, "rate_cmd") == 0 || strcmp(name, "angular_rate_cmd") == 0 ||
	    strcmp(name, "roll_rate_cmd") == 0 || strcmp(name, "roll_cmd") == 0) {
		return TOPIC_ANGULAR_RATE_CMD;
	}

	return TOPIC_INVALID;
}

static void topic_seq_bump(enum topic_id topic)
{
	if (topic >= 0 && topic < TOPIC_COUNT) {
		atomic_inc(&g_topic_seq[topic]);
	}
}

static uint32_t topic_seq_get(enum topic_id topic)
{
	if (topic < 0 || topic >= TOPIC_COUNT) {
		return 0;
	}

	return (uint32_t)atomic_get(&g_topic_seq[topic]);
}

static bool flight_state_store_copy_latest_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	uint32_t generation_start;
	uint32_t generation_end;
	uint32_t slot;
	uint16_t length;

	if (buf == NULL || len == NULL) {
		return false;
	}

	do {
		generation_start = (uint32_t)atomic_get(&g_flight_state_store.generation);
		if (generation_start == 0U) {
			return false;
		}

		slot = generation_start & 1U;
		length = g_flight_state_store.lengths[slot];
		if (length == 0U || length > buf_size) {
			return false;
		}

		memcpy(buf, g_flight_state_store.slots[slot], length);
		generation_end = (uint32_t)atomic_get(&g_flight_state_store.generation);
	} while (generation_start != generation_end);

	*len = length;
	return true;
}

static bool motor_output_store_copy_latest_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	uint32_t generation_start;
	uint32_t generation_end;
	uint32_t slot;
	uint16_t length;

	if (buf == NULL || len == NULL) {
		return false;
	}

	do {
		generation_start = (uint32_t)atomic_get(&g_motor_output_store.generation);
		if (generation_start == 0U) {
			return false;
		}

		slot = generation_start & 1U;
		length = g_motor_output_store.lengths[slot];
		if (length == 0U || length > buf_size) {
			return false;
		}

		memcpy(buf, g_motor_output_store.slots[slot], length);
		generation_end = (uint32_t)atomic_get(&g_motor_output_store.generation);
	} while (generation_start != generation_end);

	*len = length;
	return true;
}

static void flight_state_store_publish(
	const synapse_topic_Vec3f_t *gyro, const synapse_topic_Vec3f_t *accel,
	const synapse_topic_RcChannels16_t *rc, const synapse_topic_ControlStatus_t *status,
	const synapse_topic_AttitudeEuler_t *attitude,
	const synapse_topic_AttitudeEuler_t *attitude_desired,
	const synapse_topic_RateTriplet_t *rate_desired,
	const synapse_topic_RateTriplet_t *rate_cmd)
{
	size_t length;
	uint32_t next_generation;
	uint32_t slot;

	next_generation = (uint32_t)atomic_get(&g_flight_state_store.generation) + 1U;
	slot = next_generation & 1U;
	length = rdd2_topic_fb_pack_flight_state(
		g_flight_state_store.slots[slot],
		sizeof(g_flight_state_store.slots[slot]), gyro, accel, rc, status, attitude,
		attitude_desired, rate_desired, rate_cmd);
	if (length == 0U) {
		return;
	}
	g_flight_state_store.lengths[slot] = (uint16_t)length;
	atomic_set(&g_flight_state_store.generation, (atomic_val_t)next_generation);
}

static bool flight_state_store_get_latest(
	synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel,
	synapse_topic_RcChannels16_t *rc, synapse_topic_ControlStatus_t *status,
	synapse_topic_AttitudeEuler_t *attitude,
	synapse_topic_AttitudeEuler_t *attitude_desired,
	synapse_topic_RateTriplet_t *rate_desired, synapse_topic_RateTriplet_t *rate_cmd)
{
	uint8_t buf[RDD2_TOPIC_FB_FLIGHT_STATE_SIZE];
	size_t len;

	if (!flight_state_store_copy_latest_blob(buf, sizeof(buf), &len)) {
		return false;
	}

	return rdd2_topic_fb_unpack_flight_state(buf, len, gyro, accel, rc, status, attitude,
						     attitude_desired, rate_desired, rate_cmd);
}

static void motor_output_store_publish(const synapse_topic_MotorValues4f_t *motors,
				       const synapse_topic_MotorRaw4u16_t *raw, bool armed,
				       bool test_mode)
{
	size_t length;
	uint32_t next_generation;
	uint32_t slot;

	next_generation = (uint32_t)atomic_get(&g_motor_output_store.generation) + 1U;
	slot = next_generation & 1U;
	length = rdd2_topic_fb_pack_motor_output(
		g_motor_output_store.slots[slot],
		sizeof(g_motor_output_store.slots[slot]), motors, raw, armed, test_mode);
	if (length == 0U) {
		return;
	}
	g_motor_output_store.lengths[slot] = (uint16_t)length;
	atomic_set(&g_motor_output_store.generation, (atomic_val_t)next_generation);
}

static bool motor_output_store_get_latest(synapse_topic_MotorValues4f_t *motors,
					  synapse_topic_MotorRaw4u16_t *raw, bool *armed,
					  bool *test_mode)
{
	uint8_t buf[RDD2_TOPIC_FB_MOTOR_OUTPUT_SIZE];
	size_t len;

	if (!motor_output_store_copy_latest_blob(buf, sizeof(buf), &len)) {
		return false;
	}

	return rdd2_topic_fb_unpack_motor_output(buf, len, motors, raw, armed, test_mode);
}

static void topic_dynamic_get(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(g_topic_names)) {
		entry->syntax = g_topic_names[idx];
		entry->handler = NULL;
		entry->subcmd = NULL;
		entry->help = NULL;
	} else {
		entry->syntax = NULL;
	}
}

void rdd2_topic_rc_published(void)
{
	topic_seq_bump(TOPIC_RC);
}

void rdd2_topic_motor_output_publish(const synapse_topic_MotorValues4f_t *motors,
					 const synapse_topic_MotorRaw4u16_t *raw, bool armed,
					 bool test_mode)
{
	motor_output_store_publish(motors, raw, armed, test_mode);
	topic_seq_bump(TOPIC_MOTOR);
}

bool rdd2_topic_motor_output_get(synapse_topic_MotorValues4f_t *motors,
				     synapse_topic_MotorRaw4u16_t *raw, bool *armed,
				     bool *test_mode)
{
	return motor_output_store_get_latest(motors, raw, armed, test_mode);
}

bool rdd2_topic_motor_output_copy_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	return motor_output_store_copy_latest_blob(buf, buf_size, len);
}

uint32_t rdd2_topic_motor_output_generation(void)
{
	return (uint32_t)atomic_get(&g_motor_output_store.generation);
}

void rdd2_topic_flight_state_publish(
	const synapse_topic_Vec3f_t *gyro, const synapse_topic_Vec3f_t *accel,
	const synapse_topic_RcChannels16_t *rc, const synapse_topic_ControlStatus_t *status,
	const synapse_topic_AttitudeEuler_t *attitude,
	const synapse_topic_AttitudeEuler_t *attitude_desired,
	const synapse_topic_RateTriplet_t *rate_desired,
	const synapse_topic_RateTriplet_t *rate_cmd)
{
	flight_state_store_publish(gyro, accel, rc, status, attitude, attitude_desired,
				   rate_desired, rate_cmd);
	topic_seq_bump(TOPIC_STATUS);
	if (status->imu_ok) {
		topic_seq_bump(TOPIC_IMU);
	}
	topic_seq_bump(TOPIC_ATTITUDE);
	topic_seq_bump(TOPIC_ATTITUDE_DESIRED);
	topic_seq_bump(TOPIC_DESIRED_ANGULAR_RATE);
	topic_seq_bump(TOPIC_ANGULAR_RATE_CMD);
}

bool rdd2_topic_flight_state_get(
	synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel,
	synapse_topic_RcChannels16_t *rc, synapse_topic_ControlStatus_t *status,
	synapse_topic_AttitudeEuler_t *attitude,
	synapse_topic_AttitudeEuler_t *attitude_desired,
	synapse_topic_RateTriplet_t *rate_desired, synapse_topic_RateTriplet_t *rate_cmd)
{
	return flight_state_store_get_latest(gyro, accel, rc, status, attitude,
					     attitude_desired, rate_desired, rate_cmd);
}

bool rdd2_topic_flight_state_copy_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	return flight_state_store_copy_latest_blob(buf, buf_size, len);
}

uint32_t rdd2_topic_flight_state_generation(void)
{
	return (uint32_t)atomic_get(&g_flight_state_store.generation);
}

static void topic_watch_stop(void)
{
	unsigned int key = irq_lock();

	g_topic_watch.active = false;
	g_topic_watch.mode = TOPIC_WATCH_NONE;
	g_topic_watch.topic = TOPIC_INVALID;
	g_topic_watch.sh = NULL;
	g_topic_watch.period_ms = 0;
	g_topic_watch.last_seq = 0;
	g_topic_watch.last_ms = 0;

	irq_unlock(key);

	k_sem_give(&g_topic_watch_sem);
}

static void topic_watch_start(const struct shell *sh, enum topic_id topic,
			      enum topic_watch_mode mode, uint32_t period_ms)
{
	unsigned int key = irq_lock();

	g_topic_watch.sh = sh;
	g_topic_watch.topic = topic;
	g_topic_watch.mode = mode;
	g_topic_watch.period_ms = period_ms;
	g_topic_watch.last_seq = topic_seq_get(topic);
	g_topic_watch.last_ms = k_uptime_get();
	g_topic_watch.active = true;

	irq_unlock(key);

	k_sem_give(&g_topic_watch_sem);
}

static void topic_latest_flight_state(synapse_topic_Vec3f_t *gyro, synapse_topic_Vec3f_t *accel,
				      synapse_topic_RcChannels16_t *rc,
				      synapse_topic_ControlStatus_t *status,
				      synapse_topic_AttitudeEuler_t *attitude,
				      synapse_topic_AttitudeEuler_t *attitude_desired,
				      synapse_topic_RateTriplet_t *rate_desired,
				      synapse_topic_RateTriplet_t *rate_cmd)
{
	memset(gyro, 0, sizeof(*gyro));
	memset(accel, 0, sizeof(*accel));
	memset(rc, 0, sizeof(*rc));
	memset(status, 0, sizeof(*status));
	memset(attitude, 0, sizeof(*attitude));
	memset(attitude_desired, 0, sizeof(*attitude_desired));
	memset(rate_desired, 0, sizeof(*rate_desired));
	memset(rate_cmd, 0, sizeof(*rate_cmd));
	(void)rdd2_topic_flight_state_get(gyro, accel, rc, status, attitude,
					     attitude_desired, rate_desired, rate_cmd);
}

static void topic_latest_motor_output(synapse_topic_MotorValues4f_t *motors,
				      synapse_topic_MotorRaw4u16_t *raw, bool *armed,
				      bool *test_mode)
{
	memset(motors, 0, sizeof(*motors));
	memset(raw, 0, sizeof(*raw));
	*armed = false;
	*test_mode = false;
	(void)rdd2_topic_motor_output_get(motors, raw, armed, test_mode);
}

static void topic_print_status(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;
	int64_t now_ms = k_uptime_get();

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	int64_t rc_age_ms = status.rc_valid ? (now_ms - status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "status mode=%-10s armed=%1d imu_ok=%1d rc_valid=%1d rc_stale=%1d arm_switch=%1d rc_age_ms=%6lld lq=%3u throttle_us=%4ld desired_rate=[%8.3f %8.3f %8.3f] rate_cmd=[%8.3f %8.3f %8.3f]",
		    rdd2_flight_mode_name((enum rdd2_flight_mode)status.flight_mode),
		    status.armed ? 1 : 0,
		    status.imu_ok ? 1 : 0,
		    status.rc_valid ? 1 : 0,
		    status.rc_stale ? 1 : 0,
		    status.arm_switch ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)status.rc_link_quality,
		    (long)status.throttle_us,
		    (double)rate_desired.roll,
		    (double)rate_desired.pitch,
		    (double)rate_desired.yaw,
		    (double)rate_cmd.roll,
		    (double)rate_cmd.pitch,
		    (double)rate_cmd.yaw);
}

static void topic_print_imu(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	shell_print(sh,
		    "imu gyro_rad_s=" TOPIC_VEC3_FMT " accel_m_s2=" TOPIC_VEC3_FMT,
		    (double)gyro.x,
		    (double)gyro.y,
		    (double)gyro.z,
		    (double)accel.x,
		    (double)accel.y,
		    (double)accel.z);
}

static void topic_print_rc(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;
	int64_t now_ms = k_uptime_get();

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	int64_t rc_age_ms = status.rc_valid ? (now_ms - status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "rc valid=%1d age_ms=%6lld lq=%3u ch=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]",
		    status.rc_valid ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)status.rc_link_quality,
		    (long)rc.ch0,
		    (long)rc.ch1,
		    (long)rc.ch2,
		    (long)rc.ch3,
		    (long)rc.ch4,
		    (long)rc.ch5,
		    (long)rc.ch6,
		    (long)rc.ch7);
}

static void topic_print_motor(const struct shell *sh)
{
	synapse_topic_MotorValues4f_t motors;
	synapse_topic_MotorRaw4u16_t raw;
	bool armed;
	bool test_mode;

	topic_latest_motor_output(&motors, &raw, &armed, &test_mode);
	shell_print(sh,
		    "motor armed=%1d test_mode=%1d out=[%6.3f/%4u %6.3f/%4u %6.3f/%4u %6.3f/%4u]",
		    armed ? 1 : 0,
		    test_mode ? 1 : 0,
		    (double)motors.m0, (unsigned int)raw.m0,
		    (double)motors.m1, (unsigned int)raw.m1,
		    (double)motors.m2, (unsigned int)raw.m2,
		    (double)motors.m3, (unsigned int)raw.m3);
}

static void topic_print_desired_angular_rate(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	shell_print(sh, "rate_desired xyz=" TOPIC_RATE_FMT,
		    (double)rate_desired.roll,
		    (double)rate_desired.pitch,
		    (double)rate_desired.yaw);
}

static void topic_print_attitude(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	shell_print(sh, "attitude     rpy=" TOPIC_RATE_FMT,
		    (double)attitude.roll,
		    (double)attitude.pitch,
		    (double)attitude.yaw);
}

static void topic_print_attitude_desired(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	shell_print(sh, "attitude_sp  rpy=" TOPIC_RATE_FMT,
		    (double)attitude_desired.roll,
		    (double)attitude_desired.pitch,
		    (double)attitude_desired.yaw);
}

static void topic_print_angular_rate_cmd(const struct shell *sh)
{
	synapse_topic_Vec3f_t gyro;
	synapse_topic_Vec3f_t accel;
	synapse_topic_RcChannels16_t rc;
	synapse_topic_ControlStatus_t status;
	synapse_topic_AttitudeEuler_t attitude;
	synapse_topic_AttitudeEuler_t attitude_desired;
	synapse_topic_RateTriplet_t rate_desired;
	synapse_topic_RateTriplet_t rate_cmd;

	topic_latest_flight_state(&gyro, &accel, &rc, &status, &attitude, &attitude_desired,
				  &rate_desired, &rate_cmd);

	shell_print(sh, "rate_cmd     xyz=" TOPIC_RATE_FMT,
		    (double)rate_cmd.roll,
		    (double)rate_cmd.pitch,
		    (double)rate_cmd.yaw);
}

static void topic_print(const struct shell *sh, enum topic_id topic)
{
	switch (topic) {
	case TOPIC_STATUS:
		topic_print_status(sh);
		break;
	case TOPIC_IMU:
		topic_print_imu(sh);
		break;
	case TOPIC_RC:
		topic_print_rc(sh);
		break;
	case TOPIC_MOTOR:
		topic_print_motor(sh);
		break;
	case TOPIC_ATTITUDE:
		topic_print_attitude(sh);
		break;
	case TOPIC_ATTITUDE_DESIRED:
		topic_print_attitude_desired(sh);
		break;
	case TOPIC_DESIRED_ANGULAR_RATE:
		topic_print_desired_angular_rate(sh);
		break;
	case TOPIC_ANGULAR_RATE_CMD:
		topic_print_angular_rate_cmd(sh);
		break;
	default:
		shell_error(sh, "unknown topic");
		break;
	}
}

static void topic_watch_thread(void *p0, void *p1, void *p2)
{
	struct topic_watch watch;

	ARG_UNUSED(p0);
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	while (true) {
		(void)k_sem_take(&g_topic_watch_sem, K_FOREVER);

		while (true) {
			uint32_t seq_now;
			int64_t now_ms;
			unsigned int key = irq_lock();

			watch = g_topic_watch;
			irq_unlock(key);

			if (!watch.active || watch.sh == NULL || watch.topic == TOPIC_INVALID) {
				break;
			}

			if (watch.mode == TOPIC_WATCH_ECHO) {
				topic_print(watch.sh, watch.topic);
				if (k_sem_take(&g_topic_watch_sem, K_MSEC(watch.period_ms)) == 0) {
					continue;
				}
			} else if (watch.mode == TOPIC_WATCH_HZ) {
				if (k_sem_take(&g_topic_watch_sem, K_MSEC(watch.period_ms)) == 0) {
					continue;
				}

				key = irq_lock();
				watch = g_topic_watch;
				irq_unlock(key);

				if (!watch.active || watch.sh == NULL ||
				    watch.topic == TOPIC_INVALID) {
					break;
				}

				seq_now = topic_seq_get(watch.topic);
				now_ms = k_uptime_get();
				if (now_ms <= watch.last_ms) {
					now_ms = watch.last_ms + 1;
				}

				shell_print(watch.sh,
					    "%s: %u samples in %lld ms = %0.2f Hz",
					    topic_name(watch.topic),
					    (unsigned int)(seq_now - watch.last_seq),
					    (long long)(now_ms - watch.last_ms),
					    ((double)(seq_now - watch.last_seq) * 1000.0) /
						    (double)(now_ms - watch.last_ms));

				key = irq_lock();
				if (g_topic_watch.active && g_topic_watch.mode == TOPIC_WATCH_HZ &&
				    g_topic_watch.topic == watch.topic) {
					g_topic_watch.last_seq = seq_now;
					g_topic_watch.last_ms = now_ms;
				}
				irq_unlock(key);
			} else {
				break;
			}
		}
	}
}

static int parse_u32_arg(const char *arg, uint32_t *out)
{
	char *end = NULL;
	long value = strtol(arg, &end, 10);

	if (*arg == '\0' || *end != '\0' || value <= 0 || value > INT32_MAX) {
		return -EINVAL;
	}

	*out = (uint32_t)value;
	return 0;
}

static int cmd_topic_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "status");
	shell_print(sh, "imu");
	shell_print(sh, "rc");
	shell_print(sh, "motor");
	shell_print(sh, "attitude");
	shell_print(sh, "attitude_desired");
	shell_print(sh, "rate_desired");
	shell_print(sh, "rate_cmd");

	return 0;
}

static int cmd_topic_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	topic_watch_stop();
	shell_print(sh, "topic watcher stopped");

	return 0;
}

static int cmd_topic_echo(const struct shell *sh, size_t argc, char **argv)
{
	enum topic_id topic;
	uint32_t period_ms = 100;
	int rc;

	topic = topic_parse(argv[1]);
	if (topic == TOPIC_INVALID) {
		shell_error(sh, "unknown topic: %s", argv[1]);
		return -EINVAL;
	}

	if (argc >= 3) {
		rc = parse_u32_arg(argv[2], &period_ms);
		if (rc != 0) {
			shell_error(sh, "period_ms must be a positive integer");
			return rc;
		}
	}

	topic_watch_start(sh, topic, TOPIC_WATCH_ECHO, period_ms);
	shell_print(sh, "echoing %s every %u ms; use 'topic stop' to stop",
		    topic_name(topic), (unsigned int)period_ms);

	return 0;
}

static int cmd_topic_hz(const struct shell *sh, size_t argc, char **argv)
{
	enum topic_id topic;
	uint32_t period_ms = 1000;
	int rc;

	topic = topic_parse(argv[1]);
	if (topic == TOPIC_INVALID) {
		shell_error(sh, "unknown topic: %s", argv[1]);
		return -EINVAL;
	}

	if (argc >= 3) {
		rc = parse_u32_arg(argv[2], &period_ms);
		if (rc != 0) {
			shell_error(sh, "window_ms must be a positive integer");
			return rc;
		}
	}

	topic_watch_start(sh, topic, TOPIC_WATCH_HZ, period_ms);
	shell_print(sh, "measuring %s every %u ms; use 'topic stop' to stop",
		    topic_name(topic), (unsigned int)period_ms);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_topic,
	SHELL_CMD(list, NULL, "list debug topics", cmd_topic_list),
	SHELL_CMD_ARG(echo, &sub_topic_names, "echo topic: topic echo <name> [period_ms]",
		      cmd_topic_echo, 2, 1),
	SHELL_CMD_ARG(hz, &sub_topic_names, "measure topic rate: topic hz <name> [window_ms]",
		      cmd_topic_hz, 2, 1),
	SHELL_CMD(stop, NULL, "stop topic echo/hz", cmd_topic_stop),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(topic, &sub_topic, "topic-style debug commands", NULL);
