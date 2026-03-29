/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rc_input.h"
#include "topic_shell.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(cerebri2, LOG_LEVEL_INF);

#define RC_NODE    DT_ALIAS(rc)
#define IMU_NODE   DT_ALIAS(imu0)
#define DSHOT_NODE DT_ALIAS(motors)
#define GNSS_NODE  DT_ALIAS(gnss)

#define CONTROL_PERIOD_US         1000
#define RC_CHANNEL_COUNT          16
#define RC_STALE_TIMEOUT_MS       100
#define ARM_CHANNEL_INDEX         4
#define ROLL_CHANNEL_INDEX        0
#define PITCH_CHANNEL_INDEX       1
#define THROTTLE_CHANNEL_INDEX    2
#define YAW_CHANNEL_INDEX         3
#define RC_US_CENTER              1500
#define RC_US_MIN                 1000
#define RC_US_MAX                 2000
#define ARM_SWITCH_THRESHOLD_US   1600
#define THROTTLE_ARM_MAX          1050
#define IDLE_THROTTLE             0.0f
#define MAX_ROLL_PITCH_RATE_RAD_S 6.0f
#define MAX_YAW_RATE_RAD_S        3.5f
#define PID_OUTPUT_LIMIT          0.35f
#define PID_I_LIMIT               0.20f
#define DTERM_LPF_ALPHA           0.15f

struct pid_axis {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prev_measurement;
	float derivative_lpf;
};

struct motor_mix {
	float roll;
	float pitch;
	float yaw;
};

struct control_context {
	struct flight_snapshot snapshot;
	struct rc_frame rc_input;
	uint8_t rc_link_quality;
	float motors[4];
	uint16_t raw_test[4];
	float throttle_input;
	float throttle_cmd;
	float dt;
	uint32_t last_cycle;
	uint32_t now_cycle;
	int64_t now_ms;
	bool rc_stale;
	bool motor_test_active;
	bool motor_raw_test_active;
};

static atomic_t g_motor_test_active;
static float g_motor_test_values[4];
static atomic_t g_motor_raw_test_active;
static uint16_t g_motor_raw_test_values[4];

/*
 * Default mixer uses FRD body axes and assumes DSHOT channels 1..4 are:
 * front-right, rear-right, rear-left, front-left.
 *
 * Positive roll is about body +x (forward), so left motors increase and
 * right motors decrease.
 *
 * Positive pitch is about body +y (right), so front motors increase and rear
 * motors decrease.
 *
 * Yaw signs depend on motor spin direction and should be bench-verified on
 * hardware.
 */
static const struct motor_mix g_motor_mix[4] = {
	{ .roll = -1.0f, .pitch =  1.0f, .yaw = 1.0f },
	{ .roll = -1.0f, .pitch = -1.0f, .yaw =  -1.0f },
	{ .roll =  1.0f, .pitch = -1.0f, .yaw = 1.0f },
	{ .roll =  1.0f, .pitch =  1.0f, .yaw =  -1.0f },
};

static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static void rc_frame_set_defaults(struct rc_frame *frame)
{
	for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
		frame->us[i] = RC_US_CENTER;
	}

	frame->us[THROTTLE_CHANNEL_INDEX] = RC_US_MIN;
	frame->valid = false;
	frame->stamp_ms = 0;
}

static float rc_norm_centered(int32_t pulse_us)
{
	return clampf((float)(pulse_us - RC_US_CENTER) / 500.0f, -1.0f, 1.0f);
}

static float rc_norm_throttle(int32_t pulse_us)
{
	return clampf((float)(pulse_us - RC_US_MIN) / (float)(RC_US_MAX - RC_US_MIN), 0.0f, 1.0f);
}

static bool rc_switch_high(int32_t pulse_us)
{
	return pulse_us > ARM_SWITCH_THRESHOLD_US;
}

static bool imu_fetch(const struct device *imu_dev, struct imu_sample *sample)
{
	struct sensor_value gyro[3];
	struct sensor_value accel[3];
	float gyro_sensor[3];
	float accel_sensor[3];
	int rc;

	rc = sensor_sample_fetch(imu_dev);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (rc != 0) {
		return false;
	}

	for (size_t i = 0; i < 3; i++) {
		gyro_sensor[i] = sensor_value_to_float(&gyro[i]);
		accel_sensor[i] = sensor_value_to_float(&accel[i]);
	}

	/*
	 * Control code consumes IMU data in FRD body axes:
	 * body roll  (F / x) <- sensor gyro y
	 * body pitch (R / y) <- sensor gyro x
	 * body yaw   (D / z) <- -sensor gyro z
	 *
	 * Apply the same axis remap to accel so future attitude-estimator code
	 * sees a consistent body frame.
	 */
	sample->gyro_rad_s[0] = gyro_sensor[1];
	sample->gyro_rad_s[1] = gyro_sensor[0];
	sample->gyro_rad_s[2] = -gyro_sensor[2];

	sample->accel_m_s2[0] = accel_sensor[1];
	sample->accel_m_s2[1] = accel_sensor[0];
	sample->accel_m_s2[2] = -accel_sensor[2];

	return true;
}

static void motor_test_snapshot(float motors[4], bool *active)
{
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4; i++) {
		motors[i] = g_motor_test_values[i];
	}
	*active = atomic_get(&g_motor_test_active) != 0;

	irq_unlock(key);
}

static void motor_test_set(size_t index, float value)
{
	unsigned int key = irq_lock();

	g_motor_test_values[index] = clampf(value, 0.0f, 1.0f);
	atomic_set(&g_motor_test_active, 1);

	irq_unlock(key);
}

static void motor_test_clear(void)
{
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4; i++) {
		g_motor_test_values[i] = 0.0f;
	}
	atomic_set(&g_motor_test_active, 0);

	irq_unlock(key);
}

static void motor_raw_test_snapshot(uint16_t raw[4], bool *active)
{
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4; i++) {
		raw[i] = g_motor_raw_test_values[i];
	}
	*active = atomic_get(&g_motor_raw_test_active) != 0;

	irq_unlock(key);
}

static void motor_raw_test_set(size_t index, uint16_t value)
{
	unsigned int key = irq_lock();

	g_motor_raw_test_values[index] = value;
	atomic_set(&g_motor_raw_test_active, 1);

	irq_unlock(key);
}

static void motor_raw_test_set_all(uint16_t value)
{
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4; i++) {
		g_motor_raw_test_values[i] = value;
	}
	atomic_set(&g_motor_raw_test_active, 1);

	irq_unlock(key);
}

static void motor_raw_test_clear(void)
{
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4; i++) {
		g_motor_raw_test_values[i] = DSHOT_DISARMED;
	}
	atomic_set(&g_motor_raw_test_active, 0);

	irq_unlock(key);
}

static void pid_reset(struct pid_axis *pid)
{
	pid->integrator = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->derivative_lpf = 0.0f;
}

static float pid_step(struct pid_axis *pid, float setpoint, float measurement, float dt,
		      bool integrate)
{
	float error = setpoint - measurement;
	float derivative = (measurement - pid->prev_measurement) / dt;

	pid->prev_measurement = measurement;
	pid->derivative_lpf += DTERM_LPF_ALPHA * (derivative - pid->derivative_lpf);

	if (integrate) {
		pid->integrator += error * pid->ki * dt;
		pid->integrator = clampf(pid->integrator, -PID_I_LIMIT, PID_I_LIMIT);
	} else {
		pid->integrator = 0.0f;
	}

	return clampf((pid->kp * error) + pid->integrator - (pid->kd * pid->derivative_lpf),
		      -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
}

static void mix_quad_x(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
		       float out[4])
{
	float max_up = 0.0f;
	float max_down = 0.0f;
	float correction[4];
	float scale = 1.0f;

	for (size_t i = 0; i < 4; i++) {
		correction[i] = (roll_cmd * g_motor_mix[i].roll) +
				(pitch_cmd * g_motor_mix[i].pitch) +
				(yaw_cmd * g_motor_mix[i].yaw);

		if (correction[i] > max_up) {
			max_up = correction[i];
		}
		if (-correction[i] > max_down) {
			max_down = -correction[i];
		}
	}

	if (max_up > (1.0f - throttle) && max_up > 0.0f) {
		scale = (1.0f - throttle) / max_up;
	}

	if (max_down > throttle && max_down > 0.0f) {
		float down_scale = throttle / max_down;

		if (down_scale < scale) {
			scale = down_scale;
		}
	}

	for (size_t i = 0; i < 4; i++) {
		out[i] = clampf(throttle + (correction[i] * scale), 0.0f, 1.0f);
	}
}

static uint16_t motor_to_dshot(float normalized, bool armed)
{
	float min_output = armed ? IDLE_THROTTLE : 0.0f;
	float clamped = clampf(normalized, min_output, 1.0f);
	float span = (float)(DSHOT_MAX - DSHOT_MIN);

	if (!armed || clamped <= 0.0f) {
		return DSHOT_DISARMED;
	}

	return (uint16_t)(DSHOT_MIN + (clamped * span) + 0.5f);
}

static void motor_output_write_all(const float motors[4], bool armed, bool test_mode)
{
	float min_output = (armed && !test_mode) ? IDLE_THROTTLE : 0.0f;
	float applied[4];
	uint16_t raw[4];

	for (size_t i = 0; i < 4; i++) {
		applied[i] = armed ? clampf(motors[i], min_output, 1.0f) : 0.0f;
		raw[i] = motor_to_dshot(applied[i], armed && applied[i] > 0.0f);
		nxp_flexio_dshot_data_set(DEVICE_DT_GET(DSHOT_NODE), i, raw[i], false);
	}

	cerebri2_topic_motor_output_update(applied, raw, armed, test_mode);
	nxp_flexio_dshot_trigger(DEVICE_DT_GET(DSHOT_NODE));
}

static void motor_output_write_all_raw(const uint16_t raw[4], bool test_mode)
{
	float applied[4];
	uint16_t clamped[4];
	bool armed = false;

	for (size_t i = 0; i < 4; i++) {
		uint16_t value = raw[i];

		if (value != DSHOT_DISARMED && value < DSHOT_MIN) {
			value = DSHOT_MIN;
		}
		if (value > DSHOT_MAX) {
			value = DSHOT_MAX;
		}

		clamped[i] = value;
		if (value == DSHOT_DISARMED) {
			applied[i] = 0.0f;
		} else {
			applied[i] = (float)(value - DSHOT_MIN) / (float)(DSHOT_MAX - DSHOT_MIN);
			armed = true;
		}
		nxp_flexio_dshot_data_set(DEVICE_DT_GET(DSHOT_NODE), i, value, false);
	}

	cerebri2_topic_motor_output_update(applied, clamped, armed, test_mode);
	nxp_flexio_dshot_trigger(DEVICE_DT_GET(DSHOT_NODE));
}

static bool ready_or_log(const struct device *dev, const char *name)
{
	if (!device_is_ready(dev)) {
		LOG_ERR("%s not ready", name);
		return false;
	}

	return true;
}

static int control_io_init(void)
{
	const struct device *const rc_dev = DEVICE_DT_GET(RC_NODE);
	const struct device *const imu_dev = DEVICE_DT_GET(IMU_NODE);
	const struct device *const dshot_dev = DEVICE_DT_GET(DSHOT_NODE);
	const struct device *const gnss_dev = DEVICE_DT_GET_OR_NULL(GNSS_NODE);

	cerebri2_rc_input_init();

	if (!ready_or_log(dshot_dev, "dshot")) {
		return -ENODEV;
	}

	ready_or_log(rc_dev, "rc");
	ready_or_log(imu_dev, "imu");

	if (gnss_dev != NULL && device_is_ready(gnss_dev)) {
		LOG_INF("gnss path ready");
	}

	if (nxp_flexio_dshot_channel_count(dshot_dev) != 4U) {
		LOG_ERR("expected 4 dshot channels");
		return -EINVAL;
	}

	return 0;
}

static bool motor_output_ready(void)
{
	return device_is_ready(DEVICE_DT_GET(DSHOT_NODE));
}

static struct rc_frame control_input_snapshot(struct imu_sample *imu, uint8_t *rc_link_quality,
					      bool *imu_ok)
{
	*imu_ok = imu_fetch(DEVICE_DT_GET(IMU_NODE), imu);
	*rc_link_quality = cerebri2_rc_input_link_quality_get(DEVICE_DT_GET(RC_NODE));
	return cerebri2_rc_input_snapshot();
}

static void publish_flight_snapshot(const struct flight_snapshot *snapshot)
{
	cerebri2_topic_flight_snapshot_update(snapshot);
}

static int cmd_motor_status(const struct shell *sh, size_t argc, char **argv)
{
	float test_motors[4];
	float motors[4];
	uint16_t test_raw[4];
	uint16_t raw[4];
	bool active;
	bool raw_active;
	bool armed;
	bool test_mode;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_test_snapshot(test_motors, &active);
	motor_raw_test_snapshot(test_raw, &raw_active);
	cerebri2_topic_motor_output_get(motors, raw, &armed, &test_mode);
	shell_print(sh,
		    "motor_test=%d test_m0=%0.3f test_m1=%0.3f test_m2=%0.3f test_m3=%0.3f",
		    active ? 1 : 0,
		    (double)test_motors[0],
		    (double)test_motors[1],
		    (double)test_motors[2],
		    (double)test_motors[3]);
	shell_print(sh,
		    "raw_test=%d raw_m0=%u raw_m1=%u raw_m2=%u raw_m3=%u",
		    raw_active ? 1 : 0,
		    (unsigned int)test_raw[0],
		    (unsigned int)test_raw[1],
		    (unsigned int)test_raw[2],
		    (unsigned int)test_raw[3]);
	shell_print(sh,
		    "last_out armed=%d test_mode=%d m0=%0.3f/%u m1=%0.3f/%u m2=%0.3f/%u m3=%0.3f/%u",
		    armed ? 1 : 0,
		    test_mode ? 1 : 0,
		    (double)motors[0], (unsigned int)raw[0],
		    (double)motors[1], (unsigned int)raw[1],
		    (double)motors[2], (unsigned int)raw[2],
		    (double)motors[3], (unsigned int)raw[3]);
	shell_print(sh, "guess: m0=front-right m1=rear-right m2=rear-left m3=front-left");

	return 0;
}

static int cmd_fc_status(const struct shell *sh, size_t argc, char **argv)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();
	int64_t now_ms = k_uptime_get();
	int64_t rc_age_ms = snapshot.status.rc_valid ? (now_ms - snapshot.status.rc_stamp_ms) : -1;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh,
		    "armed=%d imu_ok=%d rc_valid=%d rc_stale=%d arm_switch=%d rc_age_ms=%lld lq=%u throttle_us=%ld",
		    snapshot.status.armed ? 1 : 0,
		    snapshot.status.imu_ok ? 1 : 0,
		    snapshot.status.rc_valid ? 1 : 0,
		    snapshot.status.rc_stale ? 1 : 0,
		    snapshot.status.arm_switch ? 1 : 0,
		    (long long)rc_age_ms,
		    (unsigned int)snapshot.status.rc_link_quality,
		    (long)snapshot.status.throttle_us);
	shell_print(sh,
		    "rate_desired roll=%0.3f pitch=%0.3f yaw=%0.3f rate_cmd roll=%0.3f pitch=%0.3f yaw=%0.3f",
		    (double)snapshot.rate.desired.roll,
		    (double)snapshot.rate.desired.pitch,
		    (double)snapshot.rate.desired.yaw,
		    (double)snapshot.rate.cmd.roll,
		    (double)snapshot.rate.cmd.pitch,
		    (double)snapshot.rate.cmd.yaw);

	return 0;
}

static int cmd_fc_gyro(const struct shell *sh, size_t argc, char **argv)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh,
		    "gyro_rad_s x=%0.5f y=%0.5f z=%0.5f",
		    (double)snapshot.imu.gyro_rad_s[0],
		    (double)snapshot.imu.gyro_rad_s[1],
		    (double)snapshot.imu.gyro_rad_s[2]);
	shell_print(sh,
		    "accel_m_s2 x=%0.5f y=%0.5f z=%0.5f",
		    (double)snapshot.imu.accel_m_s2[0],
		    (double)snapshot.imu.accel_m_s2[1],
		    (double)snapshot.imu.accel_m_s2[2]);

	return 0;
}

static int cmd_fc_rc(const struct shell *sh, size_t argc, char **argv)
{
	struct flight_snapshot snapshot = cerebri2_topic_flight_snapshot_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh,
		    "rc us ch0=%ld ch1=%ld ch2=%ld ch3=%ld ch4=%ld ch5=%ld ch6=%ld ch7=%ld",
		    (long)snapshot.rc.us[0],
		    (long)snapshot.rc.us[1],
		    (long)snapshot.rc.us[2],
		    (long)snapshot.rc.us[3],
		    (long)snapshot.rc.us[4],
		    (long)snapshot.rc.us[5],
		    (long)snapshot.rc.us[6],
		    (long)snapshot.rc.us[7]);

	return 0;
}

static int cmd_motor_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_test_clear();
	motor_raw_test_clear();
	if (motor_output_ready()) {
		float motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};

		motor_output_write_all(motors, false, false);
	}
	shell_print(sh, "all motor tests stopped");

	return 0;
}

static int cmd_motor_spin(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long index;
	float value;

	ARG_UNUSED(argc);

	if (!motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	index = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || index < 0 || index > 3) {
		shell_error(sh, "index must be 0..3");
		return -EINVAL;
	}

	end = NULL;
	value = strtof(argv[2], &end);
	if (*argv[2] == '\0' || *end != '\0' || value < 0.0f || value > 1.0f) {
		shell_error(sh, "value must be 0.0..1.0");
		return -EINVAL;
	}

	motor_test_set((size_t)index, value);
	shell_print(sh, "motor %ld set to %0.3f", index, (double)value);

	return 0;
}

static int cmd_motor_raw(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long index;
	long value;

	ARG_UNUSED(argc);

	if (!motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	index = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || index < 0 || index > 3) {
		shell_error(sh, "index must be 0..3");
		return -EINVAL;
	}

	end = NULL;
	value = strtol(argv[2], &end, 10);
	if (*argv[2] == '\0' || *end != '\0' || value < 0 || value > DSHOT_MAX) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}
	if (value != 0 && value < DSHOT_MIN) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}

	motor_test_clear();
	motor_raw_test_set((size_t)index, (uint16_t)value);
	shell_print(sh, "raw motor %ld set to %ld", index, value);

	return 0;
}

static int cmd_motor_raw_all(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long value;

	ARG_UNUSED(argc);

	if (!motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	value = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || value < 0 || value > DSHOT_MAX) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}
	if (value != 0 && value < DSHOT_MIN) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}

	motor_test_clear();
	motor_raw_test_set_all((uint16_t)value);
	shell_print(sh, "all raw motors set to %ld", value);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_motor,
	SHELL_CMD_ARG(spin, NULL, "spin one motor: motor spin <index:0..3> <value:0..1>",
		      cmd_motor_spin, 3, 0),
	SHELL_CMD_ARG(raw, NULL, "set one motor raw: motor raw <index:0..3> <value:0|48..2047>",
		      cmd_motor_raw, 3, 0),
	SHELL_CMD_ARG(raw_all, NULL, "set all motors raw: motor raw_all <value:0|48..2047>",
		      cmd_motor_raw_all, 2, 0),
	SHELL_CMD(stop, NULL, "stop all motor tests", cmd_motor_stop),
	SHELL_CMD(status, NULL, "show motor test state", cmd_motor_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(motor, &sub_motor, "bench motor commands", NULL);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_fc,
	SHELL_CMD(status, NULL, "show current flight-control snapshot", cmd_fc_status),
	SHELL_CMD(gyro, NULL, "show latest gyro and accel sample", cmd_fc_gyro),
	SHELL_CMD(rc, NULL, "show latest RC channel snapshot", cmd_fc_rc),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(fc, &sub_fc, "flight-control debug commands", NULL);

int main(void)
{
	struct pid_axis pid_roll = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f };
	struct pid_axis pid_pitch = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f };
	struct pid_axis pid_yaw = { .kp = 0.20f, .ki = 0.20f, .kd = 0.0000f };
	struct control_context ctx = {
		.last_cycle = k_cycle_get_32(),
	};
	int rc;

	rc_frame_set_defaults(&ctx.rc_input);
	motor_test_clear();
	motor_raw_test_clear();
	cerebri2_topic_motor_output_update((float[4]){0.0f, 0.0f, 0.0f, 0.0f},
					   (uint16_t[4]){DSHOT_DISARMED, DSHOT_DISARMED,
							 DSHOT_DISARMED, DSHOT_DISARMED},
					   false, false);

	rc = control_io_init();
	if (rc != 0) {
		return rc;
	}

	LOG_INF("cerebri2 rate-mode stack starting");

	while (true) {
		ctx.rc_input = control_input_snapshot(&ctx.snapshot.imu, &ctx.rc_link_quality,
						      &ctx.snapshot.status.imu_ok);
		ctx.now_cycle = k_cycle_get_32();
		ctx.dt = (float)k_cyc_to_us_floor32(ctx.now_cycle - ctx.last_cycle) * 1.0e-6f;
		ctx.now_ms = k_uptime_get();
		ctx.rc_stale = !ctx.rc_input.valid ||
			       ((ctx.now_ms - ctx.rc_input.stamp_ms) > RC_STALE_TIMEOUT_MS);

		ctx.last_cycle = ctx.now_cycle;
		if (ctx.dt <= 0.0f) {
			ctx.dt = (float)CONTROL_PERIOD_US * 1.0e-6f;
		}

		motor_test_snapshot(ctx.motors, &ctx.motor_test_active);
		if (ctx.motor_test_active) {
			motor_output_write_all(ctx.motors, true, true);
			k_sleep(K_USEC(CONTROL_PERIOD_US));
			continue;
		}

		motor_raw_test_snapshot(ctx.raw_test, &ctx.motor_raw_test_active);
		if (ctx.motor_raw_test_active) {
			motor_output_write_all_raw(ctx.raw_test, true);
			k_sleep(K_USEC(CONTROL_PERIOD_US));
			continue;
		}

		ctx.snapshot.status.arm_switch = rc_switch_high(ctx.rc_input.us[ARM_CHANNEL_INDEX]);

		if (!ctx.snapshot.status.imu_ok || ctx.rc_stale || !ctx.snapshot.status.arm_switch) {
			ctx.snapshot.status.armed = false;
		} else if (!ctx.snapshot.status.armed &&
			   ctx.rc_input.us[THROTTLE_CHANNEL_INDEX] <= THROTTLE_ARM_MAX) {
			ctx.snapshot.status.armed = true;
			pid_reset(&pid_roll);
			pid_reset(&pid_pitch);
			pid_reset(&pid_yaw);
		}

		for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
			ctx.snapshot.rc.us[i] = ctx.rc_input.us[i];
		}
		ctx.snapshot.status.rc_stamp_ms = ctx.rc_input.stamp_ms;
		ctx.snapshot.status.throttle_us = ctx.rc_input.us[THROTTLE_CHANNEL_INDEX];
		ctx.snapshot.status.rc_link_quality = ctx.rc_link_quality;
		ctx.snapshot.status.rc_valid = ctx.rc_input.valid;
		ctx.snapshot.status.rc_stale = ctx.rc_stale;
		ctx.snapshot.rate.desired.roll = 0.0f;
		ctx.snapshot.rate.desired.pitch = 0.0f;
		ctx.snapshot.rate.desired.yaw = 0.0f;
		ctx.snapshot.rate.cmd.roll = 0.0f;
		ctx.snapshot.rate.cmd.pitch = 0.0f;
		ctx.snapshot.rate.cmd.yaw = 0.0f;

		if (!ctx.snapshot.status.imu_ok || ctx.rc_stale) {
			publish_flight_snapshot(&ctx.snapshot);
			motor_output_write_all(ctx.motors, false, false);
			k_sleep(K_USEC(CONTROL_PERIOD_US));
			continue;
		}

		ctx.snapshot.rate.desired.roll =
			rc_norm_centered(ctx.rc_input.us[ROLL_CHANNEL_INDEX]) *
			MAX_ROLL_PITCH_RATE_RAD_S;
		ctx.snapshot.rate.desired.pitch =
			-1*rc_norm_centered(ctx.rc_input.us[PITCH_CHANNEL_INDEX]) *
			MAX_ROLL_PITCH_RATE_RAD_S;
		ctx.snapshot.rate.desired.yaw =
			rc_norm_centered(ctx.rc_input.us[YAW_CHANNEL_INDEX]) * MAX_YAW_RATE_RAD_S;

		ctx.throttle_input = rc_norm_throttle(ctx.rc_input.us[THROTTLE_CHANNEL_INDEX]);
		ctx.throttle_cmd = ctx.snapshot.status.armed
					 ? (IDLE_THROTTLE +
					    (ctx.throttle_input * (1.0f - IDLE_THROTTLE)))
					 : 0.0f;

		ctx.snapshot.rate.cmd.roll = pid_step(&pid_roll, ctx.snapshot.rate.desired.roll,
						      ctx.snapshot.imu.gyro_rad_s[0], ctx.dt,
						      ctx.snapshot.status.armed &&
							      ctx.throttle_input > 0.02f);
		ctx.snapshot.rate.cmd.pitch = pid_step(&pid_pitch, ctx.snapshot.rate.desired.pitch,
						       ctx.snapshot.imu.gyro_rad_s[1], ctx.dt,
						       ctx.snapshot.status.armed &&
							       ctx.throttle_input > 0.02f);
		ctx.snapshot.rate.cmd.yaw = pid_step(&pid_yaw, ctx.snapshot.rate.desired.yaw,
						    ctx.snapshot.imu.gyro_rad_s[2], ctx.dt,
						    ctx.snapshot.status.armed &&
							    ctx.throttle_input > 0.02f);

		mix_quad_x(ctx.throttle_cmd, ctx.snapshot.rate.cmd.roll, ctx.snapshot.rate.cmd.pitch,
			   ctx.snapshot.rate.cmd.yaw, ctx.motors);
		publish_flight_snapshot(&ctx.snapshot);
		motor_output_write_all(ctx.motors, ctx.snapshot.status.armed, false);

		k_sleep(K_USEC(CONTROL_PERIOD_US));
	}

	return 0;
}
