/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>

#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <pb_encode.h>

#include <ff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>

#include <synapse_topic_list.h>

#define FS_RET_OK FR_OK
#define MY_STACK_SIZE 8192
#define MY_PRIORITY 1
#define BUF_SIZE 8192

static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
};

LOG_MODULE_REGISTER(log_sdcard, LOG_LEVEL_DBG);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    // zros node handle
    struct zros_node node;
    // subscriptions
    struct zros_sub sub_imu;
    //struct zros_sub sub_gyro_array;
    // file
    struct fs_file_t file;
    synapse_pb_Frame frame;
    uint8_t buf[BUF_SIZE];
    // status
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    .node = {},
    .sub_imu = {},
    //.sub_gyro_array = {},
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static const char* disk_mount_pt = "/SD:";

static int mount_sd_card(void)
{
    /* raw disk i/o */
    static const char* disk_pdrv = "SD";
    uint64_t memory_size_mb;
    uint32_t block_count;
    uint32_t block_size;

    if (disk_access_init(disk_pdrv) != 0) {
        LOG_ERR("Storage init ERROR!");
        return -1;
    }

    if (disk_access_ioctl(disk_pdrv,
            DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
        LOG_ERR("Unable to get sector count");
        return -1;
    }
    LOG_INF("Block count %u", block_count);

    if (disk_access_ioctl(disk_pdrv,
            DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
        LOG_ERR("Unable to get sector size");
        return -1;
    }
    LOG_INF("Sector size %u", block_size);

    memory_size_mb = (uint64_t)block_count * block_size;
    LOG_INF("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));

    mp.mnt_point = disk_mount_pt;

    int res = fs_mount(&mp);

    if (res == FR_OK) {
        LOG_INF("Disk mounted.");
    } else {
        LOG_INF("Failed to mount disk - trying one more time");
        res = fs_mount(&mp);
        if (res != FR_OK) {
            LOG_INF("Error mounting disk.");
            return -1;
        }
    }
    return 0;
}

static int log_sdcard_init(struct context* ctx)
{
    int ret = 0;
    // initialize node
    zros_node_init(&ctx->node, "log_sdcard");

    // initialize node subscriptions
    ret = zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->frame.msg, 8000);
    if (ret < 0) {
        LOG_ERR("init imu failed: %d", ret);
        return ret;
    }

    /*
    ret = zros_sub_init(&ctx->sub_gyro_array, &ctx->node, &topic_gyro_array_0, &ctx->frame.msg, 8000);
    if (ret < 0) {
        LOG_ERR("init imu failed: %d", ret);
        return ret;
    }
    */

    ret = mount_sd_card();
    if (ret < 0) {
        return ret;
    }

    fs_file_t_init(&ctx->file);

    // delete old file if it exists
    fs_unlink("/SD:/data.pb");

    // open file
    ret = fs_open(&ctx->file, "/SD:/data.pb", FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
    if (ret < 0) {
        return ret;
    }

    k_sem_take(&ctx->running, K_FOREVER);
    LOG_INF("init");
    return ret;
};

static int log_sdcard_fini(struct context* ctx)
{
    int ret = 0;

    // close subscriptions
    zros_sub_fini(&ctx->sub_imu);
    //zros_sub_fini(&ctx->sub_gyro_array);
    zros_node_fini(&ctx->node);

    ret = fs_close(&ctx->file);
    if (ret != 0) {
        LOG_ERR("failed to close file");
    }

    ret = fs_unmount(&mp);
    if (ret < 0) {
        LOG_ERR("failed to unmount disk");
    }

    k_sem_give(&ctx->running);
    LOG_INF("fini");
    return ret;
};

static void log_sdcard_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int ret = 0;

    // constructor
    ret = log_sdcard_init(ctx);
    if (ret < 0) {
        LOG_ERR("init failed: %d", ret);
        return;
    }

    // subscribe to topics

    // while running
    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        struct k_poll_event events[] = {
            *zros_sub_get_event(&ctx->sub_imu),
            //*zros_sub_get_event(&ctx->sub_gyro_array),
        };

        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("poll timeout");
        }

        if (zros_sub_update_available(&ctx->sub_imu)) {
            zros_sub_update(&ctx->sub_imu);
            ctx->frame.which_msg = synapse_pb_Frame_imu_tag;
            pb_ostream_t stream = pb_ostream_from_buffer(ctx->buf, BUF_SIZE);
            if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, &ctx->frame, PB_ENCODE_DELIMITED)) {
                LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
            } else {
                fs_write(&ctx->file, ctx->buf, stream.bytes_written);
                // LOG_INF("logged %d bytes", stream.bytes_written);
            }
        }

        /*
        if (zros_sub_update_available(&ctx->sub_gyro_array)) {
            zros_sub_update(&ctx->sub_gyro_array);
            ctx->frame.which_msg = synapse_pb_Frame_gyro_array_0_tag;
            pb_ostream_t stream = pb_ostream_from_buffer(ctx->buf, BUF_SIZE);
            if (!pb_encode_ex(&stream, synapse_pb_Frame_fields, &ctx->frame, PB_ENCODE_DELIMITED)) {
                LOG_ERR("encoding failed: %s", PB_GET_ERROR(&stream));
            } else {
                fs_write(&ctx->file, ctx->buf, stream.bytes_written);
                // LOG_INF("logged %d bytes", stream.bytes_written);
            }
        }
        */

    }

    // deconstructor
    log_sdcard_fini(ctx);
};

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        log_sdcard_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "log_sdcard");
    k_thread_start(tid);
    return 0;
}

static int log_sdcard_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_log_sdcard, log_sdcard_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(log_sdcard, &sub_log_sdcard, "log_sdcard commands", NULL);

static int log_sdcard_sys_init(void)
{
    return start(&g_ctx);
};

//SYS_INIT(log_sdcard_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
