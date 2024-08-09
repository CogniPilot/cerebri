/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 4
#define BATCH_DURATION 50

LOG_MODULE_REGISTER(sense_accel, CONFIG_CEREBRI_SENSE_ACCEL_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// private context
struct context {
    struct zros_node node;
    synapse_pb_Vector3Array accel_array;
    synapse_pb_Vector3Array gyro_array;
    synapse_pb_Imu imu;
    struct zros_pub pub_accel_array;
    struct zros_pub pub_gyro_array;
    struct zros_pub pub_imu;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    struct rtio_sqe* streaming_handle;
    struct sensor_stream_trigger stream_trigger;
    struct sensor_read_config stream_config;
};

// private initialization
static struct context g_ctx = {
    .node = {},
    .pub_accel_array = {},
    .pub_gyro_array = {},
    .pub_imu = {},
    .imu = {
        .has_stamp = true,
        .has_angular_velocity = true,
        .has_linear_acceleration = true },
    .accel_array = synapse_pb_Vector3Array_init_default,
    .gyro_array = synapse_pb_Vector3Array_init_default,
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .streaming_handle = NULL,
    .stream_trigger = {
        .opt = SENSOR_STREAM_DATA_INCLUDE,
        .trigger = SENSOR_TRIG_FIFO_WATERMARK,
    },
    .stream_config = {
        .sensor = DEVICE_DT_GET(DT_ALIAS(accel0)),
        .is_streaming = true,
        .triggers = &g_ctx.stream_trigger,
        .count = 0,
        .max = 1,
    },
};

static RTIO_IODEV_DEFINE(iodev_accel_stream, &__sensor_iodev_api,
    &g_ctx.stream_config);

RTIO_DEFINE_WITH_MEMPOOL(accel_rtio, 4, 4, 32, 64, 4);

static int sense_accel_init(struct context* ctx)
{
    zros_node_init(&ctx->node, "sense_accel");
    zros_pub_init(&ctx->pub_accel_array, &ctx->node, &topic_accel_array_0, &ctx->accel_array);
    zros_pub_init(&ctx->pub_gyro_array, &ctx->node, &topic_gyro_array_0, &ctx->gyro_array);
    zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);

    ctx->stream_config.count = 1;

    LOG_INF("device: %p", ctx->stream_config.sensor);

    struct sensor_value val = { BATCH_DURATION, 0 };
    sensor_attr_set(
        ctx->stream_config.sensor,
        SENSOR_CHAN_ALL,
        SENSOR_ATTR_BATCH_DURATION, &val);

    sensor_attr_get(
        ctx->stream_config.sensor,
        SENSOR_CHAN_ALL,
        SENSOR_ATTR_BATCH_DURATION, &val);
    LOG_INF("set batch duration to: %d", val.val1);

    int rc = 0;

    rc = sensor_stream(&iodev_accel_stream, &accel_rtio, ctx,
        &ctx->streaming_handle);

    if (rc != 0) {
        LOG_ERR("Failed to start stream");
        return rc;
    }

    rc = k_sem_take(&ctx->running, K_FOREVER);
    if (rc != 0) {
        LOG_ERR("Failed to take running");
        return rc;
    }

    LOG_INF("init");
    return rc;
}

double q31_to_double(int32_t q31_value, int8_t shift)
{
    return ((double)q31_value) / (double)(1 << (31 - shift));
}

static void sense_accel_fini(struct context* ctx)
{
    zros_pub_fini(&ctx->pub_accel_array);
    zros_pub_fini(&ctx->pub_gyro_array);
    zros_pub_fini(&ctx->pub_imu);
    zros_node_fini(&ctx->node);

    if (ctx->streaming_handle != NULL) {
        LOG_ERR("Disabling stream");
        rtio_sqe_cancel(ctx->streaming_handle);
    }

    k_sem_give(&ctx->running);
    LOG_INF("fini");
}

static void accel_processing_callback(int result, uint8_t* buf, uint32_t buf_len, void* userdata)
{
    static uint8_t decoded_buffer[127];
    static struct {
        uint64_t base_timestamp_ns;
        int count;
        uint64_t timestamp_delta;
        int64_t values[3];
        int8_t shift;
    } accumulator_buffer;

    if (result < 0) {
        LOG_ERR("read failed");
        return;
    }

    struct context* ctx = userdata;

    if (ctx->stream_config.sensor->api == NULL) {
        LOG_ERR("sensor api is NULL");
        return;
    }

    const struct sensor_decoder_api* decoder;
    int rc = sensor_get_decoder(ctx->stream_config.sensor, &decoder);
    if (rc != 0) {
        LOG_ERR("failed to get decoder");
        return;
    }

    int channels[] = {
        SENSOR_CHAN_GYRO_XYZ,
        SENSOR_CHAN_ACCEL_XYZ,
    };
    bool gyro_updated = false;
    bool accel_updated = false;

    for (int i = 0; i < ARRAY_SIZE(channels); i++) {
        int chan = channels[i];
        struct sensor_chan_spec ch = {
            .chan_idx = 0,
            .chan_type = chan,
        };

        uint32_t fit = 0;
        memset(&accumulator_buffer, 0, sizeof(accumulator_buffer));
        while (decoder->decode(buf, ch, &fit, 1, decoded_buffer) > 0) {
            if (accumulator_buffer.count >= 127) {
                LOG_ERR("fifo overflow: %d", accumulator_buffer.count);
                break;
            }

            if (ch.chan_type == SENSOR_CHAN_GYRO_XYZ || ch.chan_type == SENSOR_CHAN_ACCEL_XYZ) {
                struct sensor_three_axis_data* data = (struct sensor_three_axis_data*)decoded_buffer;
                if (accumulator_buffer.count == 0) {
                    accumulator_buffer.base_timestamp_ns = data->header.base_timestamp_ns;
                    accumulator_buffer.timestamp_delta = data->readings[0].timestamp_delta;
                    accumulator_buffer.shift = data->shift;
                }
                accumulator_buffer.values[0] += data->readings[0].values[0];
                accumulator_buffer.values[1] += data->readings[0].values[1];
                accumulator_buffer.values[2] += data->readings[0].values[2];
                accumulator_buffer.count++;

                double x = q31_to_double(data->readings[0].values[0], data->shift);
                double y = q31_to_double(data->readings[0].values[1], data->shift);
                double z = q31_to_double(data->readings[0].values[2], data->shift);

                if (ch.chan_type == SENSOR_CHAN_GYRO_XYZ) {
                    ctx->gyro_array.value[ctx->gyro_array.value_count].x = x;
                    ctx->gyro_array.value[ctx->gyro_array.value_count].y = y;
                    ctx->gyro_array.value[ctx->gyro_array.value_count].z = z;
                    ctx->gyro_array.value_count += 1;
                } else if (ch.chan_type == SENSOR_CHAN_ACCEL_XYZ) {
                    ctx->accel_array.value[ctx->accel_array.value_count].x = x;
                    ctx->accel_array.value[ctx->accel_array.value_count].y = y;
                    ctx->accel_array.value[ctx->accel_array.value_count].z = z;
                    ctx->accel_array.value_count += 1;
                }
            }
        }

        double x = q31_to_double(accumulator_buffer.values[0] / accumulator_buffer.count, accumulator_buffer.shift);
        double y = q31_to_double(accumulator_buffer.values[1] / accumulator_buffer.count, accumulator_buffer.shift);
        double z = q31_to_double(accumulator_buffer.values[2] / accumulator_buffer.count, accumulator_buffer.shift);
        synapse_pb_Timestamp stamp;
        synapse_pb_Duration delta;
        stamp.seconds = accumulator_buffer.base_timestamp_ns / 1e9;
        stamp.nanos = accumulator_buffer.base_timestamp_ns - stamp.seconds * 1e9;
        delta.seconds = accumulator_buffer.timestamp_delta / 1e9;
        delta.nanos = accumulator_buffer.timestamp_delta - delta.seconds * 1e9;

        if (ch.chan_type == SENSOR_CHAN_GYRO_XYZ) {
            // LOG_INF("gyro %10.4f %10.4f %10.4f", x, y, z);
            ctx->gyro_array.stamp = stamp;
            ctx->gyro_array.delta = delta;
            //ctx->gyro_array.value_count = 1;
            //ctx->gyro_array.value[0].x = x;
            //ctx->gyro_array.value[0].y = y;
            //ctx->gyro_array.value[0].z = z;
            ctx->imu.angular_velocity.x = x;
            ctx->imu.angular_velocity.y = y;
            ctx->imu.angular_velocity.z = z;
            ctx->imu.stamp = stamp;
            gyro_updated = true;
        } else if (ch.chan_type == SENSOR_CHAN_ACCEL_XYZ) {
            ctx->accel_array.stamp = stamp;
            ctx->accel_array.delta = delta;
            ctx->accel_array.value_count = 1;
            ctx->accel_array.value[0].x = x;
            ctx->accel_array.value[0].y = y;
            ctx->accel_array.value[0].z = z;
            ctx->imu.linear_acceleration.x = x;
            ctx->imu.linear_acceleration.y = y;
            ctx->imu.linear_acceleration.z = z;
            ctx->imu.stamp = stamp;
            accel_updated = true;
        }
    }

    if (gyro_updated || accel_updated) {
        zros_pub_update(&ctx->pub_imu);
    }

    if (gyro_updated) {
        zros_pub_update(&ctx->pub_gyro_array);
        // LOG_INF("gyro fifo len: %d", accumulator_buffer.count);
    }

    if (accel_updated) {
        zros_pub_update(&ctx->pub_accel_array);
        // LOG_INF("accel fifo len: %d", accumulator_buffer.count);
    }
}

static void sense_accel_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    LOG_INF("starting");
    sense_accel_init(ctx);

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

        sensor_processing_with_callback(
            &accel_rtio, accel_processing_callback);

        /*
        LOG_INF("publishing");

        ctx->accel_array.has_stamp = true;
        stamp_msg(&ctx->accel_array.stamp, k_uptime_ticks());
        ctx->accel_array.value_count = 2;
        ctx->accel_array.value[0].x = 1;
        ctx->accel_array.value[0].y = 2;
        ctx->accel_array.value[0].z = 3;
        ctx->accel_array.value[1].x = 4;
        ctx->accel_array.value[1].y = 5;
        ctx->accel_array.value[1].z = 6;
        ctx->accel_array.delta.nanos = 1000;
        ctx->accel_array.delta.seconds = 0;
        zros_pub_update(&ctx->pub_accel_array);
        */
    }

    LOG_INF("finished");

    sense_accel_fini(ctx);
}

static int start(struct context* ctx)
{
    LOG_INF("star called");
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        sense_accel_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "sense_accel");
    k_thread_start(tid);
    return 0;
}

static int sense_accel_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    ARG_UNUSED(argc);
    struct context* ctx = data;

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
        shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
    }
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_accel, sense_accel_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_accel, &sub_sense_accel, "sense accel commands", NULL);

static int sense_accel_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(sense_accel_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
