/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tight rate-mode control loop copied from cerebri2 with zros pub/sub
 * for integration with the existing FSM, estimator, and logging.
 *
 * Single thread at 1000 Hz:
 *   sensor_sample_fetch → axis remap → PID → quad-X mix → dshot
 */

#include <math.h>

#include <synapse_topic_list.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/log_utils.h>

/* ── Constants (from cerebri2) ────────────────────────────────────── */

#define MY_STACK_SIZE             3072
#define MY_PRIORITY               2
#define CONTROL_PERIOD_US         1000
#define PID_OUTPUT_LIMIT          0.35f
#define PID_I_LIMIT               0.20f
#define DTERM_LPF_ALPHA           0.15f
#define IDLE_THROTTLE             0.0f
#define MAX_ROLL_PITCH_RATE_RAD_S 6.0f
#define MAX_YAW_RATE_RAD_S        3.5f
#define NUM_MOTORS                4

/* RC channel indices (AETR) */
#define CH_ROLL     0
#define CH_PITCH    1
#define CH_THROTTLE 2
#define CH_YAW      3

CEREBRI_NODE_LOG_INIT(rdd2_angular_velocity, LOG_LEVEL_WRN);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

/* ── PID (verbatim from cerebri2) ─────────────────────────────────── */

struct pid_axis {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prev_measurement;
	float derivative_lpf;
};

static float clampf(float value, float lo, float hi)
{
	if (value < lo) {
		return lo;
	}
	if (value > hi) {
		return hi;
	}
	return value;
}

static void pid_reset(struct pid_axis *pid)
{
	pid->integrator = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->derivative_lpf = 0.0f;
}

static float pid_step(struct pid_axis *pid, float setpoint, float measurement,
		      float dt, bool integrate)
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

/* ── Quad-X mixer (verbatim from cerebri2) ────────────────────────── */

/*
 * FRD body axes, DSHOT channels 1..4:
 *   m0 = front-right, m1 = rear-right, m2 = rear-left, m3 = front-left
 */
struct motor_mix {
	float roll;
	float pitch;
	float yaw;
};

static const struct motor_mix g_motor_mix[NUM_MOTORS] = {
	{ .roll = -1.0f, .pitch =  1.0f, .yaw =  1.0f },
	{ .roll = -1.0f, .pitch = -1.0f, .yaw = -1.0f },
	{ .roll =  1.0f, .pitch = -1.0f, .yaw =  1.0f },
	{ .roll =  1.0f, .pitch =  1.0f, .yaw = -1.0f },
};

static void mix_quad_x(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
		       float out[NUM_MOTORS])
{
	float max_up = 0.0f;
	float max_down = 0.0f;
	float correction[NUM_MOTORS];
	float scale = 1.0f;

	for (int i = 0; i < NUM_MOTORS; i++) {
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

	for (int i = 0; i < NUM_MOTORS; i++) {
		out[i] = clampf(throttle + (correction[i] * scale), 0.0f, 1.0f);
	}
}

static uint16_t motor_to_dshot(float normalized, bool armed)
{
	if (!armed || normalized <= 0.0f) {
		return DSHOT_DISARMED;
	}
	float clamped = clampf(normalized, 0.0f, 1.0f);

	return (uint16_t)(DSHOT_MIN + (clamped * (float)(DSHOT_MAX - DSHOT_MIN)) + 0.5f);
}

/* ── Context ──────────────────────────────────────────────────────── */

struct context {
	/* zros */
	struct zros_node node;
	synapse_pb_Status status;
	synapse_pb_Input input;
	synapse_pb_Vector3 angular_velocity_sp;
	synapse_pb_Imu imu;
	struct zros_sub sub_status, sub_input;
	struct zros_pub pub_imu, pub_angular_velocity_sp;

	/* thread */
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;

	/* devices */
	const struct device *imu_dev;
	const struct device *dshot_dev;

	/* state */
	float dt;
	float gyro[3];
	float accel[3];
	float throttle;
	float motors[NUM_MOTORS];
	struct pid_axis pid_roll;
	struct pid_axis pid_pitch;
	struct pid_axis pid_yaw;
};

static struct context g_ctx = {
	.node = {},
	.status = synapse_pb_Status_init_default,
	.input = synapse_pb_Input_init_default,
	.angular_velocity_sp = synapse_pb_Vector3_init_default,
	.imu = {
		.has_stamp = true,
		.stamp = synapse_pb_Timestamp_init_default,
		.has_angular_velocity = true,
		.angular_velocity = synapse_pb_Vector3_init_default,
		.has_linear_acceleration = true,
		.linear_acceleration = synapse_pb_Vector3_init_default,
		.has_orientation = false,
	},
	.sub_status = {},
	.sub_input = {},
	.pub_imu = {},
	.pub_angular_velocity_sp = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	/* PID gains from cerebri2 */
	.pid_roll  = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f },
	.pid_pitch = { .kp = 0.12f, .ki = 0.35f, .kd = 0.0015f },
	.pid_yaw   = { .kp = 0.20f, .ki = 0.20f, .kd = 0.0000f },
};

/* ── IMU fetch with axis remap (from cerebri2) ────────────────────── */

static bool imu_fetch(struct context *ctx)
{
	struct sensor_value gyro_sv[3];
	struct sensor_value accel_sv[3];
	float gyro_sensor[3];
	float accel_sensor[3];
	int rc;

	rc = sensor_sample_fetch(ctx->imu_dev);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(ctx->imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro_sv);
	if (rc != 0) {
		return false;
	}

	rc = sensor_channel_get(ctx->imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel_sv);
	if (rc != 0) {
		return false;
	}

	for (int i = 0; i < 3; i++) {
		gyro_sensor[i] = sensor_value_to_float(&gyro_sv[i]);
		accel_sensor[i] = sensor_value_to_float(&accel_sv[i]);
	}

	/*
	 * Axis remap from cerebri2: sensor frame → FRD body frame
	 *   body roll  (F / x) ← sensor gyro y
	 *   body pitch (R / y) ← sensor gyro x
	 *   body yaw   (D / z) ← -sensor gyro z
	 */
	ctx->gyro[0] = gyro_sensor[1];
	ctx->gyro[1] = gyro_sensor[0];
	ctx->gyro[2] = -gyro_sensor[2];

	ctx->accel[0] = accel_sensor[1];
	ctx->accel[1] = accel_sensor[0];
	ctx->accel[2] = -accel_sensor[2];

	/* Publish IMU (remapped) for estimator, FSM, and logging */
	stamp_msg(&ctx->imu.stamp, k_uptime_ticks());
	ctx->imu.angular_velocity.x = (double)ctx->gyro[0];
	ctx->imu.angular_velocity.y = (double)ctx->gyro[1];
	ctx->imu.angular_velocity.z = (double)ctx->gyro[2];
	ctx->imu.linear_acceleration.x = (double)ctx->accel[0];
	ctx->imu.linear_acceleration.y = (double)ctx->accel[1];
	ctx->imu.linear_acceleration.z = (double)ctx->accel[2];
	zros_pub_update(&ctx->pub_imu);

	return true;
}

/* ── Dshot write ──────────────────────────────────────────────────── */

static void dshot_write(struct context *ctx, bool armed)
{
	for (int i = 0; i < NUM_MOTORS; i++) {
		uint16_t raw = motor_to_dshot(ctx->motors[i], armed);

		nxp_flexio_dshot_data_set(ctx->dshot_dev, i, raw, false);
	}
	nxp_flexio_dshot_trigger(ctx->dshot_dev);
}

/* ── Init / Fini ──────────────────────────────────────────────────── */

static void rdd2_angular_velocity_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "rdd2_angular_velocity");
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_input, &ctx->node, &topic_input_rc, &ctx->input, 1000);
	zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);
	zros_pub_init(&ctx->pub_angular_velocity_sp, &ctx->node, &topic_angular_velocity_sp,
		      &ctx->angular_velocity_sp);

	ctx->imu_dev = DEVICE_DT_GET(DT_ALIAS(imu_stream_0));
	if (!device_is_ready(ctx->imu_dev)) {
		LOG_ERR("IMU device not ready");
	}

	ctx->dshot_dev = DEVICE_DT_GET(DT_NODELABEL(dshot));
	if (!device_is_ready(ctx->dshot_dev)) {
		LOG_ERR("DSHOT device not ready");
	}

	pid_reset(&ctx->pid_roll);
	pid_reset(&ctx->pid_pitch);
	pid_reset(&ctx->pid_yaw);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void rdd2_angular_velocity_fini(struct context *ctx)
{
	/* Disarm motors on exit */
	for (int i = 0; i < NUM_MOTORS; i++) {
		ctx->motors[i] = 0.0f;
	}
	dshot_write(ctx, false);

	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_input);
	zros_pub_fini(&ctx->pub_imu);
	zros_pub_fini(&ctx->pub_angular_velocity_sp);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

/* ── Main loop (matches cerebri2 main while-loop) ─────────────────── */

static void rdd2_angular_velocity_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	LOG_INF("thread starting");
	rdd2_angular_velocity_init(ctx);
	LOG_INF("init complete, entering loop");

	uint32_t last_cycle = k_cycle_get_32();

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

		/* 1. Read IMU (direct SPI, axis remapped to FRD) */
		bool imu_ok = imu_fetch(ctx);

		/* 2. Update subscriptions */
		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}
		if (zros_sub_update_available(&ctx->sub_input)) {
			zros_sub_update(&ctx->sub_input);
		}

		/* 3. Calculate dt */
		uint32_t now_cycle = k_cycle_get_32();
		ctx->dt = (float)k_cyc_to_us_floor32(now_cycle - last_cycle) * 1.0e-6f;
		last_cycle = now_cycle;
		if (ctx->dt <= 0.0f) {
			ctx->dt = (float)CONTROL_PERIOD_US * 1.0e-6f;
		}

		bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
		bool rate_mode = ctx->status.mode == synapse_pb_Status_Mode_MODE_ATTITUDE_RATE;

		/* Only drive motors in rate mode — otherwise let other threads handle it */
		if (!rate_mode || !imu_ok) {
			if (rate_mode) {
				for (int i = 0; i < NUM_MOTORS; i++) {
					ctx->motors[i] = 0.0f;
				}
				dshot_write(ctx, false);
			}
			k_usleep(CONTROL_PERIOD_US);
			continue;
		}

		/* 4. RC sticks → rate setpoints + throttle
		 *    input.channel[] is already normalized -1..+1 by sense_rc driver
		 *    Same mapping as cerebri2: AETR = ch0,ch1,ch2,ch3
		 */
		float roll_rate_sp = ctx->input.channel[CH_ROLL] * MAX_ROLL_PITCH_RATE_RAD_S;
		float pitch_rate_sp = -ctx->input.channel[CH_PITCH] * MAX_ROLL_PITCH_RATE_RAD_S;
		float yaw_rate_sp = ctx->input.channel[CH_YAW] * MAX_YAW_RATE_RAD_S;

		/* Throttle: ch2 is -1..+1, map to 0..1 */
		float throttle_input = clampf((ctx->input.channel[CH_THROTTLE] + 1.0f) * 0.5f,
					      0.0f, 1.0f);
		ctx->throttle = armed ? (IDLE_THROTTLE + throttle_input * (1.0f - IDLE_THROTTLE))
				      : 0.0f;

		/* Publish angular_velocity_sp (for FSM, logging, other consumers) */
		stamp_msg(&ctx->angular_velocity_sp.stamp, k_uptime_ticks());
		ctx->angular_velocity_sp.has_stamp = true;
		ctx->angular_velocity_sp.x = (double)roll_rate_sp;
		ctx->angular_velocity_sp.y = (double)pitch_rate_sp;
		ctx->angular_velocity_sp.z = (double)yaw_rate_sp;
		zros_pub_update(&ctx->pub_angular_velocity_sp);

		/* 5. PID on gyro (from cerebri2 pid_step) */
		float roll_cmd = pid_step(&ctx->pid_roll, roll_rate_sp, ctx->gyro[0],
					  ctx->dt, armed && throttle_input > 0.02f);
		float pitch_cmd = pid_step(&ctx->pid_pitch, pitch_rate_sp, ctx->gyro[1],
					   ctx->dt, armed && throttle_input > 0.02f);
		float yaw_cmd = pid_step(&ctx->pid_yaw, yaw_rate_sp, ctx->gyro[2],
					 ctx->dt, armed && throttle_input > 0.02f);

		if (!armed) {
			pid_reset(&ctx->pid_roll);
			pid_reset(&ctx->pid_pitch);
			pid_reset(&ctx->pid_yaw);
		}

		/* 6. Quad-X mix (from cerebri2 mix_quad_x) */
		mix_quad_x(ctx->throttle, roll_cmd, pitch_cmd, yaw_cmd, ctx->motors);

		/* 7. Dshot output (direct, no pub/sub) */
		dshot_write(ctx, armed);

		/* 8. Maintain ~1000 Hz */
		k_usleep(CONTROL_PERIOD_US);
	}

	rdd2_angular_velocity_fini(ctx);
}

/* ── Start / Shell / SysInit ──────────────────────────────────────── */

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      rdd2_angular_velocity_run, ctx, NULL, NULL, MY_PRIORITY, 0,
				      K_FOREVER);
	k_thread_name_set(tid, "rdd2_angular_velocity");
	k_thread_start(tid);
	return 0;
}

static int rdd2_angular_velocity_cmd_handler(const struct shell *sh, size_t argc, char **argv,
					     void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			k_sem_give(&g_ctx.running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running\t: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
		shell_print(sh, "dt\t: %10.6f", (double)ctx->dt);
		shell_print(sh, "gyro\t: %7.4f %7.4f %7.4f",
			    (double)ctx->gyro[0], (double)ctx->gyro[1], (double)ctx->gyro[2]);
		shell_print(sh, "throttle: %5.3f", (double)ctx->throttle);
		shell_print(sh, "motors\t: %5.3f %5.3f %5.3f %5.3f",
			    (double)ctx->motors[0], (double)ctx->motors[1],
			    (double)ctx->motors[2], (double)ctx->motors[3]);
		shell_print(sh, "roll  P=%7.4f I=%7.4f D=%7.4f",
			    (double)(ctx->pid_roll.kp *
				     ((float)ctx->angular_velocity_sp.x - ctx->gyro[0])),
			    (double)ctx->pid_roll.integrator,
			    (double)ctx->pid_roll.derivative_lpf);
		shell_print(sh, "pitch P=%7.4f I=%7.4f D=%7.4f",
			    (double)(ctx->pid_pitch.kp *
				     ((float)ctx->angular_velocity_sp.y - ctx->gyro[1])),
			    (double)ctx->pid_pitch.integrator,
			    (double)ctx->pid_pitch.derivative_lpf);
		shell_print(sh, "yaw   P=%7.4f I=%7.4f D=%7.4f",
			    (double)(ctx->pid_yaw.kp *
				     ((float)ctx->angular_velocity_sp.z - ctx->gyro[2])),
			    (double)ctx->pid_yaw.integrator,
			    (double)ctx->pid_yaw.derivative_lpf);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_rdd2_angular_velocity, rdd2_angular_velocity_cmd_handler,
			     (start, &g_ctx, "start"), (stop, &g_ctx, "stop"),
			     (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(rdd2_angular_velocity, &sub_rdd2_angular_velocity,
		   "rdd2 angular velocity commands", NULL);

static int rdd2_angular_velocity_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(rdd2_angular_velocity_sys_init, APPLICATION, 99);

// vi: ts=4 sw=4 et
