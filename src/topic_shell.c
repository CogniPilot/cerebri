/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_shell.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#define TOPIC_THREAD_STACK_SIZE 1536
#define THROTTLE_CHANNEL_INDEX  2

enum topic_id {
	TOPIC_STATUS = 0,
	TOPIC_IMU,
	TOPIC_RC,
	TOPIC_MOTOR,
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
	"imu",
	"motor",
	"rc",
	"rate_cmd",
	"rate_desired",
	"status",
};

struct flight_snapshot_store {
	struct flight_snapshot slots[2];
	atomic_t generation;
};

struct motor_output_msg {
	float motors[4];
	uint16_t raw[4];
	bool armed;
	bool test_mode;
};

struct motor_output_store {
	struct motor_output_msg slots[2];
	atomic_t generation;
};

static struct flight_snapshot_store g_flight_snapshot_store;
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

static void flight_snapshot_store_publish(const struct flight_snapshot *snapshot)
{
	uint32_t next_generation =
		(uint32_t)atomic_get(&g_flight_snapshot_store.generation) + 1U;

	g_flight_snapshot_store.slots[next_generation & 1U] = *snapshot;
	atomic_set(&g_flight_snapshot_store.generation, (atomic_val_t)next_generation);
}

static struct flight_snapshot flight_snapshot_store_snapshot(void)
{
	struct flight_snapshot snapshot;
	uint32_t generation_start;
	uint32_t generation_end;

	do {
		generation_start = (uint32_t)atomic_get(&g_flight_snapshot_store.generation);
		snapshot = g_flight_snapshot_store.slots[generation_start & 1U];
		generation_end = (uint32_t)atomic_get(&g_flight_snapshot_store.generation);
	} while (generation_start != generation_end);

	return snapshot;
}

static void motor_output_store_publish(const struct motor_output_msg *output)
{
	uint32_t next_generation =
		(uint32_t)atomic_get(&g_motor_output_store.generation) + 1U;

	g_motor_output_store.slots[next_generation & 1U] = *output;
	atomic_set(&g_motor_output_store.generation, (atomic_val_t)next_generation);
}

static struct motor_output_msg motor_output_store_snapshot(void)
{
	struct motor_output_msg output;
	uint32_t generation_start;
	uint32_t generation_end;

	do {
		generation_start = (uint32_t)atomic_get(&g_motor_output_store.generation);
		output = g_motor_output_store.slots[generation_start & 1U];
		generation_end = (uint32_t)atomic_get(&g_motor_output_store.generation);
	} while (generation_start != generation_end);

	return output;
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

void cerebri2_topic_rc_published(void)
{
	topic_seq_bump(TOPIC_RC);
}

void cerebri2_topic_motor_output_update(const float motors[4], const uint16_t raw[4], bool armed,
					bool test_mode)
{
	struct motor_output_msg output = {
		.armed = armed,
		.test_mode = test_mode,
	};

	for (size_t i = 0; i < 4; i++) {
		output.motors[i] = motors[i];
		output.raw[i] = raw[i];
	}

	motor_output_store_publish(&output);
	topic_seq_bump(TOPIC_MOTOR);
}

void cerebri2_topic_motor_output_get(float motors[4], uint16_t raw[4], bool *armed, bool *test_mode)
{
	struct motor_output_msg output = motor_output_store_snapshot();

	for (size_t i = 0; i < 4; i++) {
		motors[i] = output.motors[i];
		raw[i] = output.raw[i];
	}
	*armed = output.armed;
	*test_mode = output.test_mode;
}

void cerebri2_topic_flight_snapshot_update(const struct flight_snapshot *snapshot)
{
	flight_snapshot_store_publish(snapshot);
	topic_seq_bump(TOPIC_STATUS);
	if (snapshot->status.imu_ok) {
		topic_seq_bump(TOPIC_IMU);
	}
	topic_seq_bump(TOPIC_DESIRED_ANGULAR_RATE);
	topic_seq_bump(TOPIC_ANGULAR_RATE_CMD);
}

struct flight_snapshot cerebri2_topic_flight_snapshot_get(void)
{
	return flight_snapshot_store_snapshot();
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

static void topic_print_status(const struct shell *sh)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();
	int64_t now_ms = k_uptime_get();
	int64_t rc_age_ms =
		snapshot.status.rc_valid ? (now_ms - snapshot.status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "status armed=%d imu_ok=%d rc_valid=%d rc_stale=%d arm_switch=%d rc_age_ms=%lld lq=%u throttle_us=%ld desired_rate=[%0.3f %0.3f %0.3f] rate_cmd=[%0.3f %0.3f %0.3f]",
		    snapshot.status.armed ? 1 : 0,
		    snapshot.status.imu_ok ? 1 : 0,
		    snapshot.status.rc_valid ? 1 : 0,
		    snapshot.status.rc_stale ? 1 : 0,
		    snapshot.status.arm_switch ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)snapshot.status.rc_link_quality,
		    (long)snapshot.status.throttle_us,
		    (double)snapshot.rate.desired.roll,
		    (double)snapshot.rate.desired.pitch,
		    (double)snapshot.rate.desired.yaw,
		    (double)snapshot.rate.cmd.roll,
		    (double)snapshot.rate.cmd.pitch,
		    (double)snapshot.rate.cmd.yaw);
}

static void topic_print_imu(const struct shell *sh)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();

	shell_print(sh,
		    "imu gyro_rad_s=[%0.5f %0.5f %0.5f] accel_m_s2=[%0.5f %0.5f %0.5f]",
		    (double)snapshot.imu.gyro_rad_s[0],
		    (double)snapshot.imu.gyro_rad_s[1],
		    (double)snapshot.imu.gyro_rad_s[2],
		    (double)snapshot.imu.accel_m_s2[0],
		    (double)snapshot.imu.accel_m_s2[1],
		    (double)snapshot.imu.accel_m_s2[2]);
}

static void topic_print_rc(const struct shell *sh)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();
	int64_t now_ms = k_uptime_get();
	int64_t rc_age_ms =
		snapshot.status.rc_valid ? (now_ms - snapshot.status.rc_stamp_ms) : -1;

	shell_print(sh,
		    "rc valid=%d age_ms=%lld lq=%u ch=[%ld %ld %ld %ld %ld %ld %ld %ld]",
		    snapshot.status.rc_valid ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)snapshot.status.rc_link_quality,
		    (long)snapshot.rc.us[0],
		    (long)snapshot.rc.us[1],
		    (long)snapshot.rc.us[2],
		    (long)snapshot.rc.us[3],
		    (long)snapshot.rc.us[4],
		    (long)snapshot.rc.us[5],
		    (long)snapshot.rc.us[6],
		    (long)snapshot.rc.us[7]);
}

static void topic_print_motor(const struct shell *sh)
{
	float motors[4];
	uint16_t raw[4];
	bool armed;
	bool test_mode;

	cerebri2_topic_motor_output_get(motors, raw, &armed, &test_mode);
	shell_print(sh,
		    "motor armed=%d test_mode=%d out=[%0.3f/%u %0.3f/%u %0.3f/%u %0.3f/%u]",
		    armed ? 1 : 0,
		    test_mode ? 1 : 0,
		    (double)motors[0], (unsigned int)raw[0],
		    (double)motors[1], (unsigned int)raw[1],
		    (double)motors[2], (unsigned int)raw[2],
		    (double)motors[3], (unsigned int)raw[3]);
}

static void topic_print_desired_angular_rate(const struct shell *sh)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();

	shell_print(sh, "rate_desired { roll: %0.6f, pitch: %0.6f, yaw: %0.6f }",
		    (double)snapshot.rate.desired.roll,
		    (double)snapshot.rate.desired.pitch,
		    (double)snapshot.rate.desired.yaw);
}

static void topic_print_angular_rate_cmd(const struct shell *sh)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();

	shell_print(sh, "rate_cmd { roll: %0.6f, pitch: %0.6f, yaw: %0.6f }",
		    (double)snapshot.rate.cmd.roll,
		    (double)snapshot.rate.cmd.pitch,
		    (double)snapshot.rate.cmd.yaw);
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
