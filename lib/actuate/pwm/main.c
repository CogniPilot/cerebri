/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(actuate_pwm, CONFIG_CEREBRI_ACTUATE_PWM_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4
#define NUM_ACTUATORS DT_CHILD_NUM(DT_NODELABEL(pwm_actuators))

typedef enum pwm_type_t {
    PWM_TYPE_normalized = 0,
    PWM_TYPE_position = 1,
    PWM_TYPE_velocity = 2,
} pwm_type_t;

typedef struct actuator_pwm_t {
    const char* label;
    struct pwm_dt_spec device;
    uint32_t disarmed;
    uint32_t min;
    uint32_t max;
    uint32_t center;
    bool use_nano_seconds;
    double scale;
    uint8_t index;
    pwm_type_t type;
} actuator_pwm_t;

#define PWM_ACTUATOR_DEFINE(node_id)                                              \
    {                                                                             \
        .label = DT_NODE_FULL_NAME(node_id),                                      \
        .device = PWM_DT_SPEC_GET_BY_IDX(node_id, 0),                             \
        .disarmed = DT_PROP(node_id, disarmed_ms),                                \
        .min = DT_PROP(node_id, min_ms),                                          \
        .max = DT_PROP(node_id, max_ms),                                          \
        .center = DT_PROP(node_id, center_ms),                                    \
        .use_nano_seconds = DT_PROP(node_id, use_nano_seconds),                   \
        .scale = ((double)DT_PROP(node_id, scale)) / DT_PROP(node_id, scale_div), \
        .index = DT_NODE_CHILD_IDX(node_id),                                      \
        .type = DT_ENUM_IDX(node_id, input_type),                                 \
    },

const actuator_pwm_t g_actuator_pwms[] = {
    DT_FOREACH_CHILD(DT_NODELABEL(pwm_actuators), PWM_ACTUATOR_DEFINE)
};

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
    synapse_pb_Actuators actuators;
    synapse_pb_Status status;
    synapse_pb_Pwm pwm;
    struct zros_node node;
    struct zros_sub sub_actuators, sub_status;
    struct zros_pub pub_pwm;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    uint32_t test_pulse;
};

static struct context g_ctx = {
    .actuators = synapse_pb_Actuators_init_default,
    .status = synapse_pb_Status_init_default,
    .pwm = {
        .has_timestamp = true,
        .channel = {},
        .channel_count = NUM_ACTUATORS,
    },
    .node = {},
    .sub_status = {},
    .sub_actuators = {},
    .pub_pwm = {},
    .running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
    .stack_size = MY_STACK_SIZE,
    .stack_area = g_my_stack_area,
    .thread_data = {},
    .test_pulse = 0,
};

static int actuate_pwm_init(struct context* ctx)
{
    LOG_INF("init");
    // check pwm config
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];
        if (pwm.max < pwm.min) {
            LOG_ERR("config pwm_%d min, max must monotonically increase",
                i);
            return -1;
        }
    }

    zros_node_init(&ctx->node, "actuate_pwm");
    zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1000);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_pub_init(&ctx->pub_pwm, &ctx->node, &topic_pwm, &ctx->pwm);
    k_sem_take(&ctx->running, K_FOREVER);
    return 0;
}

static void actuate_pwm_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_sub_fini(&ctx->sub_actuators);
    zros_sub_fini(&ctx->sub_status);
    zros_pub_fini(&ctx->pub_pwm);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
}

static void pwm_update(struct context* ctx)
{
    bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
    int err = 0;

    for (int i = 0; i < NUM_ACTUATORS; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];

        uint32_t pulse = pwm.disarmed;
        double input = 0;

        if (pwm.type == PWM_TYPE_normalized) {
            input = ctx->actuators.normalized[pwm.index];
        } else if (pwm.type == PWM_TYPE_position) {
            input = ctx->actuators.position[pwm.index];
        } else if (pwm.type == PWM_TYPE_velocity) {
            input = ctx->actuators.velocity[pwm.index];
        }

        if (armed) {
            pulse = (uint32_t)((pwm.scale * input) + pwm.center);
            if (pulse > pwm.max) {
                pulse = pwm.max;
                LOG_DBG("%d  pwm saturated, requested %d > %d", pwm.index, pulse, pwm.max);
            } else if (pulse < pwm.min) {
                pulse = pwm.min;
                LOG_DBG("%d  pwm saturated, requested %d < %d", pwm.index, pulse, pwm.min);
            }
        }

        ctx->pwm.channel[i] = pulse;

        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }

        if (err) {
            LOG_ERR("failed to set pulse %d on %d (err %d)", pwm.index, pulse, err);
        }
    }

    stamp_msg(&ctx->pwm.timestamp, k_uptime_ticks());
    zros_pub_update(&ctx->pub_pwm);
}

static void actuate_pwm_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int ret = actuate_pwm_init(ctx);

    if (ret < 0) {
        return;
    }

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_actuators),
    };

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("no actuator message received");
            // put motors in disarmed state
            if (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) {
                ctx->status.arming = synapse_pb_Status_Arming_ARMING_DISARMED;
                LOG_ERR("disarming motors due to actuator msg timeout!");
            }
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_actuators)) {
            zros_sub_update(&ctx->sub_actuators);
        }

        // update pwm
        pwm_update(ctx);
    }

    actuate_pwm_fini(ctx);
}

static int start(struct context* ctx)
{
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        actuate_pwm_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "actuate_pwm");
    k_thread_start(tid);
    return 0;
}

//#if CONFIG_ACTUATE_PWM_SHELL
static int pwm_test_set_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    uint32_t pulse = atoi(argv[1]);
    LOG_INF("sending pwm %d", pulse);
    if (k_sem_count_get(&g_ctx.running) == 0) {
        shell_print(sh, "actuate_pwm running, stop it first");
        return -1;
    }
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];
        int err = 0;
        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }
        if (err) {
            LOG_ERR("failed to set pulse %d on %d (err %d)", pwm.max, pwm.index, err);
        }
    }
    return 0;
}

static int actuate_pwm_cmd_handler(const struct shell* sh,
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_actuate_pwm, actuate_pwm_cmd_handler,
    (start, &g_ctx, "start"),
    (stop, &g_ctx, "stop"),
    (status, &g_ctx, "status"));

SHELL_STATIC_SUBCMD_SET_CREATE(sub_actuate_pwm_test,
    SHELL_CMD_ARG(set, NULL, "set the pwm", pwm_test_set_handler, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(actuate_pwm, &sub_actuate_pwm, "actuate_pwm commands", NULL);
SHELL_CMD_REGISTER(actuate_pwm_test, &sub_actuate_pwm_test, "acutate_ pwm_test", NULL);

//#endif

static int actuate_pwm_sys_init(void)
{
    return start(&g_ctx);
};

SYS_INIT(actuate_pwm_sys_init, APPLICATION, 1);

/* vi: ts=4 sw=4 et */
