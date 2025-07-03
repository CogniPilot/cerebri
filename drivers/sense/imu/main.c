/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

// #include <cerebri/core/casadi.h>
#include <cerebri/core/common.h>
#include <cerebri/core/perf_duration.h>

#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

LOG_MODULE_REGISTER(sense_imu, CONFIG_CEREBRI_SENSE_IMU_LOG_LEVEL);

#define THREAD_STACK_SIZE 1024
#define THREAD_PRIORITY   6

static const double g_accel = 9.8;
static const int g_calibration_count = 100;

extern struct perf_duration control_latency;
extern struct k_work_q g_high_priority_work_q;
static void imu_work_handler(struct k_work *work);
static void imu_timer_handler(struct k_timer *dummy);

typedef struct context_t {
	// work
	struct k_work work_item;
	struct k_timer timer;
	// node
	struct zros_node node;
	// data
	synapse_pb_Imu imu;
	synapse_pb_Status status;
	synapse_pb_Status_Mode last_mode;
	bool calibrated;
	// publications
	struct zros_pub pub_imu;
	// subscriptions
	struct zros_sub sub_status;
	// gyro
	const struct device *gyro_dev;
	double gyro_raw[3];
	double gyro_bias[3];

	// accel
	const struct device *accel_dev;
	double accel_raw[3];
	double accel_bias[3];
	double accel_scale;

	// 2nd order butterworth filter states
} context_t;

static context_t g_ctx = {
	.work_item = Z_WORK_INITIALIZER(imu_work_handler),
	.timer = Z_TIMER_INITIALIZER(g_ctx.timer, imu_timer_handler, NULL),
	.node = {},
	.imu =
		{
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.has_angular_velocity = true,
			.angular_velocity = synapse_pb_Vector3_init_default,
			.has_linear_acceleration = true,
			.linear_acceleration = synapse_pb_Vector3_init_default,
			.has_orientation = false,
		},
	.status = synapse_pb_Status_init_default,
	.last_mode = synapse_pb_Status_Mode_MODE_UNKNOWN,
	.calibrated = false,
	.pub_imu = {},
	.sub_status = {},
	.gyro_dev = NULL,
	.gyro_raw = {},
	.gyro_bias = {},
	.accel_dev = NULL,
	.accel_raw = {},
	.accel_bias = {},
};

static void imu_init(context_t *ctx)
{
	LOG_INF("init");

	// initialize node
	zros_node_init(&ctx->node, "sense_imu");
	zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);

	// setup accel devices
	ctx->accel_dev = get_device(DEVICE_DT_GET(DT_ALIAS(accel0)));

	// setup gyro devices
	ctx->gyro_dev = get_device(DEVICE_DT_GET(DT_ALIAS(gyro0)));
}

void imu_read(context_t *ctx)
{
	// default all data to zero
	struct sensor_value accel_value[3] = {};
	struct sensor_value gyro_value[3] = {};

	// get accel if device present
	if (ctx->accel_dev != NULL) {
		sensor_sample_fetch(ctx->accel_dev);
		sensor_channel_get(ctx->accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_value);
		for (int j = 0; j < 3; j++) {
			ctx->accel_raw[j] = accel_value[j].val1 + accel_value[j].val2 * 1e-6;

			if (ctx->accel_raw[j] > 15 * g_accel || ctx->accel_raw[j] < -15 * g_accel) {
				LOG_ERR("accel saturating: %d: %10.4f", j, ctx->accel_raw[j]);
				continue;
			}
		}
	}

	// get gyro if device present
	if (ctx->gyro_dev != NULL) {
		// don't resample if it is the same device as accel, want same timestamp
		if (ctx->gyro_dev != ctx->accel_dev) {
			sensor_sample_fetch(ctx->gyro_dev);
		}
		sensor_channel_get(ctx->gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro_value);
		for (int j = 0; j < 3; j++) {
			ctx->gyro_raw[j] = gyro_value[j].val1 + gyro_value[j].val2 * 1e-6;

			if (ctx->gyro_raw[j] > 34 || ctx->gyro_raw[j] < -34) {
				LOG_ERR("gyro saturating: %d: %10.4f", j, ctx->gyro_raw[j]);
				continue;
			}
		}
	}
}

void imu_calibrate(context_t *ctx)
{
	// data
	double accel_samples[g_calibration_count][3];
	double gyro_samples[g_calibration_count][3];

	// mean and std
	double accel_mean[3];
	double gyro_mean[3];
	double accel_std[3];
	double gyro_std[3];

	// repeat until calibrated
	while (true) {

		LOG_INF("calibration started, keep device stationary");

		// reset sum to zero
		memset(gyro_samples, 0, sizeof(gyro_samples));
		memset(accel_samples, 0, sizeof(accel_samples));
		memset(accel_mean, 0, sizeof(accel_mean));
		memset(accel_std, 0, sizeof(accel_std));
		memset(gyro_mean, 0, sizeof(gyro_mean));
		memset(gyro_std, 0, sizeof(gyro_std));

		// attempt to get samples
		for (int i = 0; i < g_calibration_count; i++) {
			k_msleep(5);
			imu_read(ctx);

			// get accel data
			for (int k = 0; k < 3; k++) {
				accel_samples[i][k] = ctx->accel_raw[k];
				accel_mean[k] += accel_samples[i][k];
			}

			// get gyro data
			for (int k = 0; k < 3; k++) {
				gyro_samples[i][k] = ctx->gyro_raw[k];
				gyro_mean[k] += gyro_samples[i][k];
			}
		}

		// find accel mean
		for (int k = 0; k < 3; k++) {
			accel_mean[k] /= g_calibration_count;
		}

		// find gyro mean
		for (int k = 0; k < 3; k++) {
			gyro_mean[k] /= g_calibration_count;
		}

		// find std deviation
		for (int i = 0; i < g_calibration_count; i++) {
			for (int k = 0; k < 3; k++) {
				double e = accel_samples[i][k] - accel_mean[k];
				accel_std[k] += e * e;
			}

			// get gyro data
			for (int k = 0; k < 3; k++) {
				double e = gyro_samples[i][k] - gyro_mean[k];
				gyro_std[k] += e * e;
			}
		}

		for (int k = 0; k < 3; k++) {
			accel_std[k] = sqrt(accel_std[k] / g_calibration_count);
		}

		for (int k = 0; k < 3; k++) {
			gyro_std[k] = sqrt(gyro_std[k] / g_calibration_count);
		}

		// check if calibration acceptable
		bool calibration_ok = true;
		
		// Check if acceleration magnitude is reasonable (should be close to 9.8 m/s²)
		double accel_magnitude = sqrt(accel_mean[0] * accel_mean[0] + 
					     accel_mean[1] * accel_mean[1] + 
					     accel_mean[2] * accel_mean[2]);
		
		if (accel_magnitude < 8.0 || accel_magnitude > 11.0) {
			LOG_WRN("accel magnitude out of range: %10.4f (expected ~9.8)", accel_magnitude);
			calibration_ok = false;
		}
		
		// Check if gyro readings are stable (low std deviation)
		for (int k = 0; k < 3; k++) {
			if (gyro_std[k] > 0.1) {  // 0.1 rad/s threshold
				LOG_WRN("gyro axis %d too noisy: std=%10.4f", k, gyro_std[k]);
				calibration_ok = false;
			}
		}
		
		if (calibration_ok) {
			break;
		} else {
			LOG_INF("calibration failed, retrying...");
			k_msleep(1000);  // Wait before retry
		}
	}

	LOG_INF("calibration completed");
	ctx->accel_bias[0] = accel_mean[0];
	ctx->accel_bias[1] = accel_mean[1];
	ctx->accel_bias[2] = 0;
	ctx->accel_scale = sqrt(accel_mean[0] * accel_mean[0] + accel_mean[1] * accel_mean[1] +
				accel_mean[2] * accel_mean[2]) /
			   g_accel;
	LOG_INF("accel");
	LOG_INF("mean: %10.4f %10.4f %10.4f", accel_mean[0], accel_mean[1], accel_mean[2]);
	LOG_INF("std: %10.4f %10.4f %10.4f", accel_std[0], accel_std[1], accel_std[2]);
	LOG_INF("scale %10.4f", ctx->accel_scale);

	LOG_INF("gyro");
	LOG_INF("mean: %10.4f %10.4f %10.4f", gyro_mean[0], gyro_mean[1], gyro_mean[2]);
	LOG_INF("std: %10.4f %10.4f %10.4f", gyro_std[0], gyro_std[1], gyro_std[2]);
	for (int k = 0; k < 3; k++) {
		ctx->gyro_bias[k] = gyro_mean[k];
	}
	ctx->calibrated = true;
}

void imu_publish(context_t *ctx)
{
	// update message
	stamp_msg(&ctx->imu.stamp, k_uptime_ticks());
	ctx->imu.angular_velocity.x = ctx->gyro_raw[0] - ctx->gyro_bias[0];
	ctx->imu.angular_velocity.y = ctx->gyro_raw[1] - ctx->gyro_bias[1];
	ctx->imu.angular_velocity.z = ctx->gyro_raw[2] - ctx->gyro_bias[2];
	ctx->imu.linear_acceleration.x =
		(ctx->accel_raw[0] - ctx->accel_bias[0]) / ctx->accel_scale;
	ctx->imu.linear_acceleration.y =
		(ctx->accel_raw[1] - ctx->accel_bias[1]) / ctx->accel_scale;
	ctx->imu.linear_acceleration.z =
		(ctx->accel_raw[2] - ctx->accel_bias[2]) / ctx->accel_scale;

	// publish message
	zros_pub_update(&ctx->pub_imu);
	// LOG_INF("publish imu");
}

void imu_work_handler(struct k_work *work)
{
	context_t *ctx = CONTAINER_OF(work, context_t, work_item);

	// update status
	if (zros_sub_update_available(&ctx->sub_status)) {
		zros_sub_update(&ctx->sub_status);
	}

	// handle calibration request
	if (ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION &&
	    ctx->last_mode != synapse_pb_Status_Mode_MODE_CALIBRATION) {
		ctx->calibrated = false;
	}
	ctx->last_mode = ctx->status.mode;

	if (!ctx->calibrated) {
		LOG_INF("calibrating");
		imu_calibrate(ctx);
		return;
	}

	perf_duration_start(&control_latency);
	imu_read(ctx);
	imu_publish(ctx);
}

void imu_timer_handler(struct k_timer *timer)
{
	context_t *ctx = CONTAINER_OF(timer, context_t, timer);
	k_work_submit_to_queue(&g_high_priority_work_q, &ctx->work_item);
}

int sense_imu_entry_point(context_t *ctx)
{
	imu_init(ctx);
	// delay initiali calibration 1 s
	k_msleep(1000);
	k_timer_start(&ctx->timer, K_MSEC(5), K_MSEC(5));
	return 0;
}

K_THREAD_DEFINE(sense_imu, THREAD_STACK_SIZE, sense_imu_entry_point, &g_ctx, NULL, NULL,
		THREAD_PRIORITY, 0, 100);

// vi: ts=4 sw=4 et
