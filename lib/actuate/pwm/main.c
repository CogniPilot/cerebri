/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuator_pwm.h"
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

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

extern actuator_pwm_t g_actuator_pwms[];

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
        .channel_count = CONFIG_CEREBRI_ACTUATE_PWM_NUMBER,
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
    for (int i = 0; i < CONFIG_CEREBRI_ACTUATE_PWM_NUMBER; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];
        if (pwm.max < pwm.center || pwm.min > pwm.center) {
            LOG_ERR("config pwm_%d min, center, "
                    "max must monotonically increase",
                i);
            return -1;
        }
    }

    zros_node_init(&ctx->node, "actuate_pwm");
    zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 100);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 100);
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

static void pwm_update(struct context * ctx)
{
    bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
    int err = 0;

    for (int i = 0; i < CONFIG_CEREBRI_ACTUATE_PWM_NUMBER; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];

        uint32_t pulse = pwm.disarmed;
        if (armed) {
            if (pwm.type == PWM_TYPE_NORMALIZED) {
                float input = armed ? ctx->actuators.normalized[pwm.index] : 0;
                if (input < -1 || input > 1) {
                    LOG_ERR("normalized input out of bounds");
                    continue;
                }
                if (input > 0) {
                    pulse += input * (pwm.max - pwm.center);
                } else {
                    pulse += input * (pwm.center - pwm.min);
                }
                LOG_DBG("%s position index %d with input %f pulse %d", pwm.alias, pwm.index, (double)input, pulse);
            } else if (pwm.type == PWM_TYPE_POSITION) {
                float input = armed ? ctx->actuators.position[pwm.index] : 0;
                uint32_t output = (uint32_t)((pwm.slope * input) + pwm.intercept);
                if (output > pwm.max) {
                    pulse = pwm.max;
                    LOG_DBG("%s  position command saturated, requested %d > %d", pwm.alias, output, pwm.max);
                } else if (output < pwm.min) {
                    pulse = pwm.min;
                    LOG_DBG("%s  position command saturated, requested %d < %d", pwm.alias, output, pwm.min);
                } else {
                    pulse = output;
                }
                LOG_DBG("%s position index %d with input %f output %d pulse %d",
                    pwm.alias, pwm.index, (double)input, output, pulse);
            } else if (pwm.type == PWM_TYPE_VELOCITY) {
                float input = armed ? ctx->actuators.velocity[pwm.index] : 0;
                uint32_t output = (uint32_t)((pwm.slope * input) + pwm.intercept);
                if (output > pwm.max) {
                    pulse = pwm.max;
                    LOG_DBG("%s velocity command saturated, requested %d > %d\n", pwm.alias, output, pwm.max);
                } else if (output < pwm.min) {
                    pulse = pwm.min;
                    LOG_DBG("%s  velocity command saturated, requested %d < %d\n", pwm.alias, output, pwm.min);
                } else {
                    pulse = output;
                }
                LOG_DBG("%s  velocity index %d with input %f output %d pulse %d\n",
                    pwm.alias, pwm.index, (double)input, output, pulse);
            }
        }
        ctx->pwm.channel[i] = pulse;

        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }

        if (err) {
            LOG_ERR("failed to set pulse %d on %s (err %d)", pulse, pwm.alias, err);
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
    for (int i = 0; i < CONFIG_CEREBRI_ACTUATE_PWM_NUMBER; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];
        int err = 0;
        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }
        if (err) {
            LOG_ERR("failed to set pulse %d on %s (err %d)", pwm.max, pwm.alias, err);
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
