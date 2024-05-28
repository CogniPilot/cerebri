/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <zephyr/input/input.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(sense_joy, CONFIG_CEREBRI_SENSE_JOY_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

// jextern struct k_work_q g_high_priority_work_q;
// static void sense_joy_work_handler(struct k_work* work);
// static void sense_joy_timer_handler(struct k_timer* dummy);

struct context {
    // struct k_work work_item;
    // struct k_timer timer;
    struct zros_node node;
    struct zros_pub pub_joy;
    synapse_msgs_Joy joy;
    atomic_t running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    struct k_sem sem;
    int counter;
};

static struct context g_ctx = {
    //.work_item = Z_WORK_INITIALIZER(sense_joy_work_handler),
    //.timer = Z_TIMER_INITIALIZER(g_ctx.timer, sense_joy_timer_handler, NULL),
    .running = ATOMIC_INIT(0),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .sem = {},
    .joy = {
        .axes_count = 5,
        .buttons_count = 8,
        .axes = {},
        .buttons = {},
    },
    .counter = 0,
};

/*
void sense_joy_work_handler(struct k_work* work)
{
    // struct context* ctx = CONTAINER_OF(work, struct context, work_item);
}

void sense_joy_timer_handler(struct k_timer* timer)
{
    // struct context* ctx = CONTAINER_OF(timer, struct context, timer);
    // k_work_submit_to_queue(&g_high_priority_work_q, &ctx->work_item);
}
*/

static void sense_joy_init(struct context* ctx)
{
    LOG_INF("init");
    zros_node_init(&ctx->node, "sense_joy");
    zros_pub_init(&ctx->pub_joy, &ctx->node, &topic_joy, &ctx->joy);
    k_sem_init(&ctx->sem, 1, 1);
    atomic_set(&ctx->running, 1);
}

static void sense_joy_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_node_fini(&ctx->node);
    zros_pub_fini(&ctx->pub_joy);
    atomic_set(&ctx->running, 0);
}

static void sense_joy_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    sense_joy_init(ctx);

    while (atomic_get(&ctx->running)) {
        k_msleep(1000);
        // k_msleep(100);
        // g_ctx.joy.buttons[JOY_BUTTON_ARM] = 0;
        // g_ctx.joy.buttons[JOY_BUTTON_DISARM] = 0;
        // zros_pub_update(&ctx->pub_joy);
    }

    sense_joy_fini(ctx);
}

static int start(struct context* ctx)
{
    k_msleep(3000);
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        sense_joy_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "sense_joy");
    k_thread_start(tid);
    return 0;
}

enum joy_code {
    CODE_YAW = 0,
    CODE_THROTTLE = 1,
    CODE_ROLL = 3,
    CODE_PITCH = 4,
    CODE_KILL = 11,
};

static void input_cb(struct input_event* evt)
{
    if (!atomic_get(&g_ctx.running)) {
        return;
    }
    double x0 = 1024;
    double scale = 784;

    k_sem_take(&g_ctx.sem, K_FOREVER);
    if (evt->code == CODE_THROTTLE) {
        g_ctx.joy.axes[JOY_AXES_THRUST] = (evt->value - x0) / scale;
    } else if (evt->code == CODE_YAW) {
        g_ctx.joy.axes[JOY_AXES_YAW] = (evt->value - x0) / scale;
    } else if (evt->code == CODE_ROLL) {
        g_ctx.joy.axes[JOY_AXES_ROLL] = (evt->value - x0) / scale;
    } else if (evt->code == CODE_PITCH) {
        g_ctx.joy.axes[JOY_AXES_PITCH] = (evt->value - x0) / scale;
    } else if (evt->code == CODE_KILL) {
        if (evt->value == 1) {
            g_ctx.joy.buttons[JOY_BUTTON_ARM] = 1;
            g_ctx.joy.buttons[JOY_BUTTON_DISARM] = 0;
        } else if (evt->value == 0) {
            g_ctx.joy.buttons[JOY_BUTTON_DISARM] = 1;
            g_ctx.joy.buttons[JOY_BUTTON_ARM] = 0;
        }
    } else {
        LOG_INF("unhandled event: %d %d %d %d", evt->code, evt->sync, evt->type, evt->value);
    }

    if (g_ctx.counter < 50) {
        g_ctx.counter += 1;
    } else {
        g_ctx.counter = 0;
        zros_pub_update(&g_ctx.pub_joy);
    }
    k_sem_give(&g_ctx.sem);
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

static int sense_joy_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    struct context* ctx = data;
    __ASSERT(argc == 1, "one argument allowed");

    if (strcmp(argv[0], "start") == 0) {
        if (atomic_get(&ctx->running)) {
            shell_print(sh, "already running");
        } else {
            start(ctx);
        }
    } else if (strcmp(argv[0], "stop") == 0) {
        if (atomic_get(&ctx->running)) {
            atomic_set(&ctx->running, 0);
        } else {
            shell_print(sh, "not running");
        }
    } else if (strcmp(argv[0], "status") == 0) {
        shell_print(sh, "running: %d", (int)atomic_get(&ctx->running));
    }
    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_joy, sense_joy_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_joy, &sub_sense_joy, "sense joy args", NULL);

static int sense_joy_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(sense_joy_sys_init, APPLICATION, 2);

/* vi: ts=4 sw=4 et */
