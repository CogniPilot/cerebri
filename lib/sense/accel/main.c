/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/sensor.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(sense_accel, CONFIG_CEREBRI_SENSE_ACCEL_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

/* Create a single common config for streaming */
static struct sensor_stream_trigger iodev_accel_trigger = {
    .opt = SENSOR_STREAM_DATA_INCLUDE,
    .trigger = SENSOR_TRIG_FIFO_WATERMARK,
};
static struct sensor_read_config iodev_accel_stream_config = {
	.sensor = NULL,
	.is_streaming = true,
	.triggers = &iodev_accel_trigger,
	.count = 0,
	.max = 1,
};

RTIO_IODEV_DEFINE(iodev_accel_stream, &__sensor_iodev_api,
		  &iodev_accel_stream_config);

RTIO_DEFINE_WITH_MEMPOOL(accel_rtio, 8, 8, 32, 64, 4);

// private context
struct context {
    struct zros_node node;
    synapse_pb_Vector3Array accel_array;
    struct zros_pub pub_accel_array;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
	struct rtio_sqe *streaming_handle;
};

// private initialization
static struct context g_ctx = {
    .node = {},
    .accel_array = synapse_pb_Vector3Array_init_default,
    .pub_accel_array = {},
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static void sense_accel_init(struct context* ctx)
{
    zros_node_init(&ctx->node, "sense_accel");
    zros_pub_init(&ctx->pub_accel_array, &ctx->node, &topic_accel_array_0, &ctx->accel_array);

	int rc = sensor_stream(&iodev_accel_stream, &accel_rtio, &ctx,
			       &ctx->streaming_handle);

	if (rc != 0) {
		LOG_ERR("Failed to start stream");
	}

    k_sem_take(&ctx->running, K_FOREVER);

    LOG_INF("init");
}

static void sense_accel_fini(struct context* ctx)
{
    zros_pub_fini(&ctx->pub_accel_array);
    zros_node_fini(&ctx->node);

	if (ctx->streaming_handle != NULL) {
		LOG_ERR("Disabling stream");
		rtio_sqe_cancel(ctx->streaming_handle);
	}

    k_sem_give(&ctx->running);
    LOG_INF("fini");
}

static void accel_processing_callback(int result, uint8_t *buf, uint32_t buf_len, void *userdata)
{
    struct context *ctx = userdata;
    LOG_INF("got data");
}

static void sense_accel_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    sense_accel_init(ctx);

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

        k_msleep(1000);

		sensor_processing_with_callback(
            &accel_rtio, accel_processing_callback);

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
    }

    sense_accel_fini(ctx);
}

static int start(struct context* ctx)
{
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
