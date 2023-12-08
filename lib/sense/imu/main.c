/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>

#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

LOG_MODULE_REGISTER(sense_imu, CONFIG_CEREBRI_SENSE_IMU_LOG_LEVEL);

#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 6

static const double g_accel = 9.8;
static const int g_calibration_count = 100;

extern struct k_work_q g_high_priority_work_q;
void imu_work_handler(struct k_work* work);
void imu_timer_handler(struct k_timer* dummy);

typedef struct context_t {
    // work
    struct k_work work_item;
    struct k_timer timer;
    // node
    struct zros_node node;
    // data
    synapse_msgs_Imu imu;
    synapse_msgs_Status status;
    synapse_msgs_Status_Mode last_mode;
    bool calibrated;
    // publications
    struct zros_pub pub_imu;
    // subscriptions
    struct zros_sub sub_status;
    // devices
    const struct device* accel_dev[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT];
    const struct device* gyro_dev[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT];
    // raw readings
    double gyro_raw[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3];
    double accel_raw[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3];
    // bias
    double gyro_bias[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3];
    double accel_bias[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3];
    double accel_scale[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT];
} context_t;

static context_t g_ctx = {
    .work_item = Z_WORK_INITIALIZER(imu_work_handler),
    .timer = Z_TIMER_INITIALIZER(g_ctx.timer, imu_timer_handler, NULL),
    .node = {},
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
    .status = synapse_msgs_Status_init_default,
    .last_mode = synapse_msgs_Status_Mode_MODE_UNKNOWN,
    .calibrated = false,
    .pub_imu = {},
    .sub_status = {},
    .accel_dev = {},
    .gyro_dev = {},
    .gyro_raw = {},
    .accel_raw = {},
    .gyro_bias = {},
    .accel_bias = {},
};

static void imu_init(context_t* ctx)
{
    // initialize node
    zros_node_init(&ctx->node, "sense_imu");
    zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);

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

void imu_read(context_t* ctx)
{
    for (int i = 0; i < MAX(CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT,
                        CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT);
         i++) {
        // default all data to zero
        struct sensor_value accel_value[3] = {};
        struct sensor_value gyro_value[3] = {};

        // get accel if device present
        if (i < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT) {
            if (ctx->accel_dev[i] != NULL) {
                sensor_sample_fetch(ctx->accel_dev[i]);
                sensor_channel_get(ctx->accel_dev[i], SENSOR_CHAN_ACCEL_XYZ, accel_value);
                for (int j = 0; j < 3; j++) {
                    ctx->accel_raw[i][j] = accel_value[j].val1 + accel_value[j].val2 * 1e-6;
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
                    ctx->gyro_raw[i][j] = gyro_value[j].val1 + gyro_value[j].val2 * 1e-6;
                }
                LOG_DBG("gyro %d: %d.%06d %d.%06d %d.%06d", i,
                    gyro_value[0].val1, gyro_value[0].val2,
                    gyro_value[1].val1, gyro_value[1].val2,
                    gyro_value[2].val1, gyro_value[2].val2);
            }
        }
    }
}

void imu_calibrate(context_t* ctx)
{
    // data
    double accel_samples[g_calibration_count][CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3];
    double gyro_samples[g_calibration_count][CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3];

    // mean and std
    double accel_mean[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3];
    double gyro_mean[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3];
    double accel_std[CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT][3];
    double gyro_std[CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT][3];

    // repeat until calibrated
    while (true) {

        LOG_INF("calibration started, keep level, don't move");

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
            for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT; j++) {
                for (int k = 0; k < 3; k++) {
                    accel_samples[i][j][k] = ctx->accel_raw[j][k];
                    accel_mean[j][k] += accel_samples[i][j][k];
                }
            }

            // get gyro data
            for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT; j++) {
                for (int k = 0; k < 3; k++) {
                    gyro_samples[i][j][k] = ctx->gyro_raw[j][k];
                    gyro_mean[j][k] += gyro_samples[i][j][k];
                }
            }
        }

        // find accel mean
        for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT; j++) {
            for (int k = 0; k < 3; k++) {
                accel_mean[j][k] /= g_calibration_count;
            }
        }

        // find gyro mean
        for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT; j++) {
            for (int k = 0; k < 3; k++) {
                gyro_mean[j][k] /= g_calibration_count;
            }
        }

        // find std deviation
        for (int i = 0; i < g_calibration_count; i++) {
            // get accel data
            for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT; j++) {
                for (int k = 0; k < 3; k++) {
                    double e = accel_samples[i][j][k] - accel_mean[j][k];
                    accel_std[j][k] += e * e;
                }
            }

            // get gyro data
            for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT; j++) {
                for (int k = 0; k < 3; k++) {
                    double e = gyro_samples[i][j][k] - gyro_mean[j][k];
                    gyro_std[j][k] += e * e;
                }
            }
        }

        for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT; j++) {
            for (int k = 0; k < 3; k++) {
                accel_std[j][k] = sqrt(accel_std[j][k] / g_calibration_count);
            }
        }

        for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT; j++) {
            for (int k = 0; k < 3; k++) {
                gyro_std[j][k] = sqrt(gyro_std[j][k] / g_calibration_count);
            }
        }

        // check if calibration acceptable, and break if it is
        // TODO implement calibration check
        break;
    }

    LOG_INF("calibration completed");
    for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_ACCEL_COUNT; j++) {
        ctx->accel_bias[j][0] = accel_mean[j][0];
        ctx->accel_bias[j][1] = accel_mean[j][1];
        ctx->accel_bias[j][2] = 0;
        ctx->accel_scale[j] = sqrt(
                                  accel_mean[j][0] * accel_mean[j][0] + accel_mean[j][1] * accel_mean[j][1] + accel_mean[j][2] * accel_mean[j][2])
            / g_accel;
        LOG_INF("accel %d", j);
        LOG_INF("mean: %10.4f %10.4f %10.4f", accel_mean[j][0], accel_mean[j][1], accel_mean[j][2]);
        LOG_INF("std: %10.4f %10.4f %10.4f", accel_std[j][0], accel_std[j][1], accel_std[j][2]);
        LOG_INF("scale %10.4f", ctx->accel_scale[j]);
    }

    for (int j = 0; j < CONFIG_CEREBRI_SENSE_IMU_GYRO_COUNT; j++) {
        LOG_INF("gyro %d", j);
        LOG_INF("mean: %10.4f %10.4f %10.4f", gyro_mean[j][0], gyro_mean[j][1], gyro_mean[j][2]);
        LOG_INF("std: %10.4f %10.4f %10.4f", gyro_std[j][0], gyro_std[j][1], gyro_std[j][2]);
        for (int k = 0; k < 3; k++) {
            ctx->gyro_bias[j][k] = gyro_mean[j][k];
        }
    }
    ctx->calibrated = true;
}

void imu_publish(context_t* ctx)
{
    // TODO implement voting
    static const int accel_select = 0;
    static const int gyro_select = 0;

    // update message
    stamp_header(&ctx->imu.header, k_uptime_ticks());
    ctx->imu.header.seq++;
    ctx->imu.angular_velocity.x = ctx->gyro_raw[gyro_select][0] - ctx->gyro_bias[gyro_select][0];
    ctx->imu.angular_velocity.y = ctx->gyro_raw[gyro_select][1] - ctx->gyro_bias[gyro_select][1];
    ctx->imu.angular_velocity.z = ctx->gyro_raw[gyro_select][2] - ctx->gyro_bias[gyro_select][2];
    ctx->imu.linear_acceleration.x = (ctx->accel_raw[accel_select][0] - ctx->accel_bias[accel_select][0]) / ctx->accel_scale[accel_select];
    ctx->imu.linear_acceleration.y = (ctx->accel_raw[accel_select][1] - ctx->accel_bias[accel_select][1]) / ctx->accel_scale[accel_select];
    ctx->imu.linear_acceleration.z = (ctx->accel_raw[accel_select][2] - ctx->accel_bias[accel_select][2]) / ctx->accel_scale[accel_select];

    // publish message
    zros_pub_update(&ctx->pub_imu);
    // LOG_INF("publish imu");
}

void imu_work_handler(struct k_work* work)
{
    context_t* ctx = CONTAINER_OF(work, context_t, work_item);

    // update status
    if (zros_sub_update_available(&ctx->sub_status)) {
        zros_sub_update(&ctx->sub_status);
    }

    // handle calibration request
    if (ctx->status.mode == synapse_msgs_Status_Mode_MODE_CALIBRATION && ctx->last_mode != synapse_msgs_Status_Mode_MODE_CALIBRATION) {
        ctx->calibrated = false;
    }
    ctx->last_mode = ctx->status.mode;

    if (!ctx->calibrated) {
        LOG_INF("calibrating");
        imu_calibrate(ctx);
        return;
    }

    imu_read(ctx);
    imu_publish(ctx);
}

void imu_timer_handler(struct k_timer* timer)
{
    context_t* ctx = CONTAINER_OF(timer, context_t, timer);
    k_work_submit_to_queue(&g_high_priority_work_q, &ctx->work_item);
}

int sense_imu_entry_point(context_t* ctx)
{
    imu_init(ctx);
    k_timer_start(&ctx->timer, K_MSEC(5), K_MSEC(5));
    return 0;
}

K_THREAD_DEFINE(sense_imu, THREAD_STACK_SIZE,
    sense_imu_entry_point, &g_ctx, NULL, NULL,
    THREAD_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
