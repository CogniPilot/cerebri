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
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(actuate_pwm, CONFIG_CEREBRI_ACTUATE_PWM_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

extern actuator_pwm_t g_actuator_pwms[];

typedef struct _context {
    synapse_msgs_Actuators actuators;
    synapse_msgs_Status status;
    struct zros_node node;
    struct zros_sub sub_actuators, sub_status;
    struct pwm_dt_spec pwm_enable;
} context;

static context g_ctx = {
    .actuators = synapse_msgs_Actuators_init_default,
    .status = synapse_msgs_Status_init_default,
    .node = {},
    .sub_status = {},
    .sub_actuators = {},
    .pwm_enable = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux2)),
};

static void actuate_pwm_init(context* ctx)
{
    zros_node_init(&ctx->node, "actuate_pwm");
    zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 100);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 100);
}

void pwm_update(const synapse_msgs_Status* status, const synapse_msgs_Actuators* actuators)
{
    bool armed = status->arming == synapse_msgs_Status_Arming_ARMING_ARMED;
    int err = 0;

    if (armed) {
        err = pwm_set_pulse_dt(&g_ctx.pwm_enable, PWM_USEC(50));
    } else {
        err = pwm_set_pulse_dt(&g_ctx.pwm_enable, PWM_USEC(0));
    }

    for (int i = 0; i < CONFIG_CEREBRI_ACTUATE_PWM_NUMBER; i++) {
        actuator_pwm_t pwm = g_actuator_pwms[i];
        if (pwm.max < pwm.center || pwm.min > pwm.center) {
            LOG_ERR("config pwm_%d min, center, "
                    "max must monotonically increase",
                i);
            continue;
        }
        uint32_t pulse = pwm.center;
        if (pwm.type == PWM_TYPE_NORMALIZED) {
            float input = armed ? actuators->normalized[pwm.index] : 0;
            if (input < -1 || input > 1) {
                LOG_ERR("normalized input out of bounds");
                continue;
            }
            if (input > 0) {
                pulse += input * (pwm.max - pwm.center);
            } else {
                pulse += input * (pwm.center - pwm.min);
            }
        } else if (pwm.type == PWM_TYPE_POSITION) {
            float input = armed ? actuators->position[pwm.index] : 0;
            uint32_t output = (uint32_t)((pwm.slope * input) + pwm.intercept);
            LOG_DBG("%s position index %d with input %f output %d", pwm.alias, pwm.index, input, output);
            if (output > pwm.max) {
                pulse = pwm.max;
                LOG_DBG("%s  position command saturated, requested %d > %d", pwm.alias, output, pwm.max);
            } else if (output < pwm.min) {
                pulse = pwm.min;
                LOG_DBG("%s  position command saturated, requested %d < %d", pwm.alias, output, pwm.min);
            } else {
                pulse = output;
            }
        } else if (pwm.type == PWM_TYPE_VELOCITY) {
            float input = armed ? actuators->velocity[pwm.index] : 0;
            uint32_t output = (uint32_t)((pwm.slope * input) + pwm.intercept);
            LOG_DBG("%s  velocity index %d with input %f output %d\n", pwm.alias, pwm.index, input, output);
            if (output > pwm.max) {
                pulse = pwm.max;
                LOG_DBG("%s velocity command saturated, requested %d > %d\n", pwm.alias, output, pwm.max);
            } else if (output < pwm.min) {
                pulse = pwm.min;
                LOG_DBG("%s  velocity command saturated, requested %d < %d\n", pwm.alias, output, pwm.min);
            } else {
                pulse = output;
            }
        }

        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }

        if (err) {
            LOG_ERR("failed to set pulse %d on %s (err %d)", pulse, pwm.alias, err);
        }
    }
}

void actuate_pwm_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    actuate_pwm_init(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_actuators),
    };

    while (true) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("no actuator message received");
            // put motors in disarmed state
            if (ctx->status.arming == synapse_msgs_Status_Arming_ARMING_ARMED) {
                ctx->status.arming = synapse_msgs_Status_Arming_ARMING_DISARMED;
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
        pwm_update(&ctx->status, &ctx->actuators);
    }
}

K_THREAD_DEFINE(actuate_pwm, MY_STACK_SIZE,
    actuate_pwm_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
