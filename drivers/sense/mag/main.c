/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(sense_mag, CONFIG_CEREBRI_SENSE_MAG_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   6

extern struct k_work_q g_high_priority_work_q;
void mag_work_handler(struct k_work *work);

typedef struct context {
	struct k_work work_item;
	const struct device *device[CONFIG_CEREBRI_SENSE_MAG_COUNT];
	struct zros_node node;
	struct zros_pub pub;
	synapse_pb_MagneticField data;
} context_t;

static context_t g_ctx = {.work_item = Z_WORK_INITIALIZER(mag_work_handler),
			  .device = {},
			  .node = {},
			  .pub = {},
			  .data = {
				  .has_stamp = true,
				  .frame_id = "base_link",
				  .stamp = synapse_pb_Timestamp_init_default,
				  .magnetic_field = synapse_pb_Vector3_init_default,
				  .has_magnetic_field = true,
				  .magnetic_field_covariance = {},
				  .magnetic_field_covariance_count = 0,
			  }};

void mag_work_handler(struct k_work *work)
{
	context_t *ctx = CONTAINER_OF(work, context_t, work_item);
	double mag_data_array[CONFIG_CEREBRI_SENSE_MAG_COUNT][3] = {};
	for (int i = 0; i < CONFIG_CEREBRI_SENSE_MAG_COUNT; i++) {
		// default all data to zero
		struct sensor_value mag_value[3] = {};

		// get accel if device present
		if (ctx->device[i] != NULL) {
			sensor_sample_fetch(ctx->device[i]);
			sensor_channel_get(ctx->device[i], SENSOR_CHAN_MAGN_XYZ, mag_value);
			LOG_DBG("mag %d: %d.%06d %d.%06d %d.%06d", i, mag_value[0].val1,
				mag_value[0].val2, mag_value[1].val1, mag_value[1].val2,
				mag_value[2].val1, mag_value[2].val2);
		}

		for (int j = 0; j < 3; j++) {
			mag_data_array[i][j] = mag_value[j].val1 + mag_value[j].val2 * 1e-6;
		}
	}

	// select first mag for data for now: TODO implement voting
	double mag[3] = {mag_data_array[0][1], -mag_data_array[0][0], -mag_data_array[0][2]};

	// Define calibration parameters
	double A[3][3] = {
		{1.9720, 0.0109, 0.1156},
		{0.0000, 1.9013, -0.0532},
		{0.0000, 0.0000, 1.9597}
	};
	
	double b[3] = {0.0012, 0.0327, 0.0111};

	double temp[3];

	// Subtract bias
    for (int i = 0; i < 3; i++){
		mag[i] -= b[i];
	}

    // Apply calibration matrix
    for (int i = 0; i < 3; i++) {
        temp[i] = 0;
        for (int j = 0; j < 3; j++)
            temp[i] += A[i][j] * mag[j];
    }

    // Copy calibrated data back
    for (int i = 0; i < 3; i++){
		mag[i] = temp[i];
	}

	// publish
	stamp_msg(&ctx->data.stamp, k_uptime_ticks());
	ctx->data.magnetic_field.x = mag[0];
	ctx->data.magnetic_field.y = mag[1];
	ctx->data.magnetic_field.z = mag[2];
	zros_pub_update(&ctx->pub);
}

void mag_timer_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&g_high_priority_work_q, &g_ctx.work_item);
}

K_TIMER_DEFINE(mag_timer, mag_timer_handler, NULL);

int sense_mag_entry_point(context_t *ctx)
{
	LOG_INF("init");
	ctx->device[0] = get_device(DEVICE_DT_GET(DT_ALIAS(mag0)));
#if CONFIG_CEREBRI_SENSE_MAG_COUNT >= 2
	ctx->device[1] = get_device(DEVICE_DT_GET(DT_ALIAS(mag1)));
#elif CONFIG_CEREBRI_SENSE_MAG_COUNT >= 3
	ctx->device[2] = get_device(DEVICE_DT_GET(DT_ALIAS(mag2)));
#elif CONFIG_CEREBRI_SENSE_MAG_COUNT >= 4
	ctx->device[3] = get_device(DEVICE_DT_GET(DT_ALIAS(mag3)));
#endif

	zros_node_init(&ctx->node, "sense_mag");
	zros_pub_init(&ctx->pub, &ctx->node, &topic_magnetic_field, &ctx->data);
	k_timer_start(&mag_timer, K_MSEC(20), K_MSEC(20));
	return 0;
}

K_THREAD_DEFINE(sense_mag, MY_STACK_SIZE, sense_mag_entry_point, &g_ctx, NULL, NULL, MY_PRIORITY, 0,
		100);

// vi: ts=4 sw=4 et
