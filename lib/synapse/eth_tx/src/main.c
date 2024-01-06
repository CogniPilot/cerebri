#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>

#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <pb_encode.h>

#include "proto/udp_tx.h"

#include <synapse_topic_list.h>

#include <synapse_tinyframe/SynapseTopics.h>
#include <synapse_tinyframe/TinyFrame.h>
#include <synapse_tinyframe/utils.h>

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 1

LOG_MODULE_REGISTER(syn_eth_tx, LOG_LEVEL_DBG);

#define TOPIC_PUBLISHER(DATA, CLASS, TOPIC)                                   \
    {                                                                         \
        TF_Msg msg;                                                           \
        TF_ClearMsg(&msg);                                                    \
        uint8_t buf[CLASS##_size];                                            \
        memset(buf, 0, CLASS##_size);                                         \
        pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));  \
        int rc = pb_encode(&stream, CLASS##_fields, DATA);                    \
        if (rc) {                                                             \
            msg.type = TOPIC;                                                 \
            msg.data = buf;                                                   \
            msg.len = stream.bytes_written;                                   \
            TF_Send(&ctx->tf, &msg);                                          \
        } else {                                                              \
            printf("%s encoding failed: %s\n", #DATA, PB_GET_ERROR(&stream)); \
        }                                                                     \
    }

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);
static struct k_thread g_my_thread_data;

struct context {
    // zros node handle
    struct zros_node node;
    // subscriptions
    struct zros_sub
        sub_actuators,
        sub_estimator_odometry,
        sub_nav_sat_fix,
        sub_status;
    // topic data
    synapse_msgs_Actuators actuators;
    synapse_msgs_NavSatFix nav_sat_fix;
    synapse_msgs_Odometry estimator_odometry;
    synapse_msgs_Status status;
    // connections
    struct udp_tx udp;
    // tinyframe
    TinyFrame tf;
    // status
    atomic_t running;
};

static struct context g_ctx;

static void tf_write(TinyFrame* tf, const uint8_t* buf, uint32_t len)
{
    struct context* ctx = tf->userdata;
    udp_tx_send(&ctx->udp, buf, len);
}

static void send_uptime(struct context* ctx)
{
    TF_Msg msg;
    TF_ClearMsg(&msg);
    uint8_t buf[synapse_msgs_Time_size];
    pb_ostream_t stream = pb_ostream_from_buffer((pu8)buf, sizeof(buf));
    int64_t ticks = k_uptime_ticks();
    int64_t sec = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    synapse_msgs_Time message;
    message.sec = sec;
    message.nanosec = nanosec;
    int rc = pb_encode(&stream, synapse_msgs_Time_fields, &message);
    if (rc) {
        msg.type = SYNAPSE_UPTIME_TOPIC;
        msg.data = buf;
        msg.len = stream.bytes_written;
        TF_Send(&ctx->tf, &msg);
    } else {
        printf("uptime encoding failed: %s\n", PB_GET_ERROR(&stream));
    }
}

static int init(struct context* ctx)
{
    int ret = 0;
    // initialize node
    zros_node_init(&ctx->node, "syn_eth_tx");

    // initialize node subscriptions
    ret = zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 15);
    if (ret < 0) {
        LOG_ERR("init actuators failed: %d", ret);
        return ret;
    }
    ret = zros_sub_init(&ctx->sub_estimator_odometry, &ctx->node, &topic_estimator_odometry, &ctx->estimator_odometry, 15);
    if (ret < 0) {
        LOG_ERR("sub init estimator odometry failed: %d", ret);
        return ret;
    }
    ret = zros_sub_init(&ctx->sub_nav_sat_fix, &ctx->node, &topic_nav_sat_fix, &ctx->nav_sat_fix, 15);
    if (ret < 0) {
        LOG_ERR("sub init nav_sat_fix failed: %d", ret);
        return ret;
    }
    ret = zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 15);
    if (ret < 0) {
        LOG_ERR("sub init status failed: %d", ret);
        return ret;
    }

    // initialize udp
    ret = udp_tx_init(&ctx->udp);
    if (ret < 0) {
        LOG_ERR("udp init failed: %d", ret);
        return ret;
    }

    // setup tinyframe
    ret = TF_InitStatic(&ctx->tf, TF_MASTER, tf_write);
    if (ret < 0) {
        LOG_ERR("tf init failed: %d", ret);
        return ret;
    }
    ctx->tf.userdata = ctx;

    // set running to true
    ctx->running = ATOMIC_INIT(1);
    return ret;
};

static int fini(struct context* ctx)
{
    int ret = 0;
    ret = udp_tx_fini(&ctx->udp);

    // close subscriptions
    zros_sub_fini(&ctx->sub_actuators);
    zros_sub_fini(&ctx->sub_estimator_odometry);
    zros_sub_fini(&ctx->sub_nav_sat_fix);
    zros_sub_fini(&ctx->sub_status);

    return ret;
};

static void run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int ret = 0;

    // constructor
    ret = init(ctx);
    if (ret < 0) {
        LOG_ERR("init failed: %d", ret);
        return;
    }

    int64_t ticks_last_uptime = 0;

    LOG_INF("running");

    // subscribe to topics

    // while running
    while (atomic_get(&ctx->running)) {
        int64_t now = k_uptime_ticks();

        struct k_poll_event events[] = {
            *zros_sub_get_event(&ctx->sub_status),
            *zros_sub_get_event(&ctx->sub_estimator_odometry),
            *zros_sub_get_event(&ctx->sub_nav_sat_fix),
        };

        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_WRN("poll timeout");
        }

        if (zros_sub_update_available(&ctx->sub_nav_sat_fix)) {
            zros_sub_update(&ctx->sub_nav_sat_fix);
            TOPIC_PUBLISHER(&ctx->nav_sat_fix, synapse_msgs_NavSatFix, SYNAPSE_NAV_SAT_FIX_TOPIC);
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
            TOPIC_PUBLISHER(&ctx->status, synapse_msgs_Status, SYNAPSE_STATUS_TOPIC);
        }

        if (zros_sub_update_available(&ctx->sub_estimator_odometry)) {
            zros_sub_update(&ctx->sub_estimator_odometry);
            TOPIC_PUBLISHER(&ctx->estimator_odometry, synapse_msgs_Odometry, SYNAPSE_ODOMETRY_TOPIC);
        }

        if (now - ticks_last_uptime > CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
            send_uptime(ctx);
            ticks_last_uptime = now;
        }

        // tell tinyframe time has passed
        TF_Tick(&ctx->tf);
    }

    // deconstructor
    ret = fini(ctx);
    if (ret < 0) {
        LOG_ERR("fini failed: %d", ret);
    }
};

static int start()
{
    k_tid_t tid = k_thread_create(&g_my_thread_data, g_my_stack_area,
        K_THREAD_STACK_SIZEOF(g_my_stack_area),
        run,
        &g_ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "syn_eth_tx");
    k_thread_start(tid);
    return 0;
}

static int cmd_start(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    if (atomic_get(&g_ctx.running)) {
        shell_print(sh, "already running");
    } else {
        start();
    }
    return 0;
}

static int cmd_stop(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    if (atomic_get(&g_ctx.running)) {
        atomic_set(&g_ctx.running, 0);
    } else {
        shell_print(sh, "not running");
    }
    return 0;
}

static int cmd_status(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    shell_print(sh, "running: %d", (int)atomic_get(&g_ctx.running));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_syn_eth_tx,
    SHELL_CMD(start, NULL, "start", cmd_start),
    SHELL_CMD(stop, NULL, "stop", cmd_stop),
    SHELL_CMD(status, NULL, "status", cmd_status),
    SHELL_SUBCMD_SET_END);

/* Creating root (level 0) command "demo" */
SHELL_CMD_REGISTER(syn_eth_tx, &sub_syn_eth_tx, "syn eth tx commands", NULL);

SYS_INIT(start, APPLICATION, 0);

// vi: ts=4 sw=4 et
