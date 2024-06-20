/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <ff.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 1

LOG_MODULE_REGISTER(synapse_sdcard, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
};

/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */
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

#define WRITE_BUF_SIZE 2000

struct context {
    // node
    struct zros_node node;
    // subscriptinos
    struct zros_sub sub_imu;
    // data
    synapse_msgs_Imu imu;
    struct fs_file_t file;
    char file_data_buffer[WRITE_BUF_SIZE];
    // threading
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
};

static struct context g_ctx = {
    // node
    .node = {},
    // subscriptions
    .sub_imu = {},
    // data
    .imu = synapse_msgs_Imu_init_default,
    .file = {},
    .file_data_buffer = {},
    // threading
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
};

static int synapse_sdcard_init(struct context* ctx)
{
    zros_node_init(&ctx->node, "synapse_sdcard");
    zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu, &ctx->imu, 300);

    int rc = mount_sd_card();
    if (rc < 0)
        return rc;

    fs_file_t_init(&ctx->file);
    k_sem_take(&ctx->running, K_FOREVER);
    return 0;
};

static void synapse_sdcard_fini(struct context* ctx)
{
    zros_sub_fini(&ctx->sub_imu);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
};

static void synapse_sdcard_run(void* p0, void* p1, void* p2)
{
    int rc = 0;
    int count = 0;
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    rc = synapse_sdcard_init(ctx);
    if (rc < 0)
        return;

    // delete old file if it exists
    fs_unlink("/SD:/partial_data.txt");

    // open file
    rc = fs_open(&ctx->file, "/SD:/partial_data.txt", FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
    if (rc < 0)
        return;

    struct k_poll_event events[1] = {};

    // while running
    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        // poll for imu
        events[0] = *zros_sub_get_event(&ctx->sub_imu);
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("not receiving imu");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_imu)) {
            zros_sub_update(&ctx->sub_imu);
            count++;

            int n = snprintf(ctx->file_data_buffer,
                WRITE_BUF_SIZE,
                "%10.4f %10.4f %10.4f\n",
                ctx->imu.angular_velocity.x,
                ctx->imu.angular_velocity.y,
                ctx->imu.angular_velocity.z);

            rc = fs_write(&ctx->file, ctx->file_data_buffer, n);
            if (rc < 0) {
                LOG_ERR("write failed");
            }
            if (count > 1000) {
                count = 0;
                rc = fs_sync(&ctx->file);
                if (rc < 0) {
                    LOG_ERR("sync failed");
                }
            }
        }
    }

    synapse_sdcard_fini(ctx);
};

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        synapse_sdcard_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "synapse_sdcard");
    k_thread_start(tid);
    return 0;
}

static int synapse_sdcard_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_synapse_sdcard, synapse_sdcard_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(synapse_sdcard, &sub_synapse_sdcard, "synapse_sdcard commands", NULL);

static int synapse_sdcard_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(synapse_sdcard_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
