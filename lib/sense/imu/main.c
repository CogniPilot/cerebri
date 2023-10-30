/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>
#include <cerebri/synapse/zbus/channels.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

LOG_MODULE_REGISTER(sense_imu, CONFIG_CEREBRI_SENSE_IMU_LOG_LEVEL);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 6

extern struct k_work_q g_high_priority_work_q;

typedef struct _context {
    // node
    syn_node_t node;
    // data
    synapse_msgs_Imu imu;
    // publications
    syn_pub_t pub_imu;
    // devices
    const struct device* accel_dev[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT];
    const struct device* gyro_dev[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT];
} context;

static context g_ctx = {
    .node = { 0 },
    .imu = {
        .has_header = true,
        .header = {
            .frame_id = "base_link",
            .has_stamp = true,
            .seq = 0 },
        .has_angular_velocity = true,
        .angular_velocity = synapse_msgs_Vector3_init_default,
        .has_linear_acceleration = true,
        .linear_acceleration = synapse_msgs_Vector3_init_default,
        .has_orientation = false,
    },
    .pub_imu = { 0 },
    .accel_dev = { 0 },
    .gyro_dev = { 0 },
};

static void imu_init(context* ctx)
{
    // initialize node
    syn_node_init(&ctx->node, "imu");
    syn_node_add_pub(&ctx->node, &ctx->pub_imu, &ctx->imu, &chan_out_imu);

    // setup accel devices

    ctx->accel_dev[0] = get_device(DEVICE_DT_GET(DT_ALIAS(accel0)));
#if CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT >= 2
    ctx->accel_dev[1] = get_device(DEVICE_DT_GET(DT_ALIAS(accel1)));
#elif CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT >= 3
    ctx->accel_dev[2] = get_device(DEVICE_DT_GET(DT_ALIAS(accel2)));
#elif CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT == 4
    ctx->accel_dev[3] = get_device(DEVICE_DT_GET(DT_ALIAS(accel3)));
#endif

    // setup gyro devices
    ctx->gyro_dev[0] = get_device(DEVICE_DT_GET(DT_ALIAS(gyro0)));
#if CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT >= 2
    ctx->gyro_dev[1] = get_device(DEVICE_DT_GET(DT_ALIAS(gyro1)));
#elif CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT >= 3
    ctx->gyro_dev[2] = get_device(DEVICE_DT_GET(DT_ALIAS(gyro2)));
#elif CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT == 4
    ctx->gyro_dev[3] = get_device(DEVICE_DT_GET(DT_ALIAS(gyro3)));
#endif
}

void imu_work_handler(struct k_work* work)
{
    context* ctx = &g_ctx;

    double accel_data_array[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3] = { 0 };
    double gyro_data_array[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3] = { 0 };

    //        LOG_DBG("");
    for (int i = 0; i < MAX(CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT,
        CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT); i++) {
        // default all data to zero
        struct sensor_value accel_value[3] = {};
        struct sensor_value gyro_value[3] = {};

        // get accel if device present
        if (i < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT) {
            if (ctx->accel_dev[i] != NULL) {
                sensor_sample_fetch(ctx->accel_dev[i]);
                sensor_channel_get(ctx->accel_dev[i], SENSOR_CHAN_ACCEL_XYZ, accel_value);
                for (int j = 0; j < 3; j++) {
                    accel_data_array[i][j] = accel_value[j].val1 + accel_value[j].val2 * 1e-6;
                }
                LOG_DBG("accel %d: %d.%06d %d.%06d %d.%06d", i,
                    accel_value[0].val1, accel_value[0].val2,
                    accel_value[1].val1, accel_value[1].val2,
                    accel_value[2].val1, accel_value[2].val2);

            }
        }

        // get gyro if device present
        if (i < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT) {
            if (ctx->gyro_dev[i] != NULL) {
                // don't resample if it is the same device as accel, want same timestamp
                if (ctx->gyro_dev[i] != ctx->accel_dev[i]) {
                    sensor_sample_fetch(ctx->gyro_dev[i]);
                }
                sensor_channel_get(ctx->gyro_dev[i], SENSOR_CHAN_GYRO_XYZ, gyro_value);
                for (int j = 0; j < 3; j++) {
                    gyro_data_array[i][j] = gyro_value[j].val1 + gyro_value[j].val2 * 1e-6;
                }
                LOG_DBG("gyro %d: %d.%06d %d.%06d %d.%06d", i,
                    gyro_value[0].val1, gyro_value[0].val2,
                    gyro_value[1].val1, gyro_value[1].val2,
                    gyro_value[2].val1, gyro_value[2].val2);
            }
        }
    }

    // select first imu for data for now: TODO implement voting
    double accel[3] = {
        accel_data_array[0][0],
        accel_data_array[0][1],
        accel_data_array[0][2]
    };

    double gyro[3] = {
        gyro_data_array[0][0],
        gyro_data_array[0][1],
        gyro_data_array[0][2]
    };

    // lock message
    syn_node_lock_all(&ctx->node, K_MSEC(1));

    // update message
    stamp_header(&ctx->imu.header, k_uptime_ticks());
    ctx->imu.header.seq++;
    ctx->imu.angular_velocity.x = gyro[0];
    ctx->imu.angular_velocity.y = gyro[1];
    ctx->imu.angular_velocity.z = gyro[2];
    ctx->imu.linear_acceleration.x = accel[0];
    ctx->imu.linear_acceleration.y = accel[1];
    ctx->imu.linear_acceleration.z = accel[2];

    // publish message
    syn_node_publish_all(&ctx->node, K_MSEC(1));
    // LOG_INF("publish imu");

    // unlock message
    syn_node_unlock_all(&ctx->node);
}

K_WORK_DEFINE(imu_work, imu_work_handler);

void imu_timer_handler(struct k_timer* dummy)
{
    k_work_submit_to_queue(&g_high_priority_work_q, &imu_work);
}

K_TIMER_DEFINE(imu_timer, imu_timer_handler, NULL);

int sense_imu_entry_point(context* ctx)
{
    imu_init(ctx);
    k_timer_start(&imu_timer, K_MSEC(5), K_MSEC(5));
    return 0;
}

K_THREAD_DEFINE(sense_imu, THREAD_STACK_SIZE,
    sense_imu_entry_point, &g_ctx, NULL, NULL,
    THREAD_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
