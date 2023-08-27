/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuator_pwm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <cerebri/synapse/zbus/channels.h>
#include <cerebri/synapse/zbus/syn_pub_sub.h>

LOG_MODULE_REGISTER(actuate_pwm, CONFIG_CEREBRI_ACTUATE_PWM_LOG_LEVEL);

extern struct k_work_q g_high_priority_work_q;

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

extern actuator_pwm_t g_actuator_pwms[];

typedef struct _context {
    synapse_msgs_Actuators actuators;
    synapse_msgs_Fsm fsm;
    syn_sub_t sub_actuators, sub_fsm;
} context;

static context g_ctx = {
    .actuators = synapse_msgs_Actuators_init_default,
    .fsm = synapse_msgs_Fsm_init_default,
    .sub_fsm = { 0 },
};

static void init(context* ctx)
{
    syn_sub_init(&ctx->sub_actuators, &ctx->actuators, &chan_out_actuators);
    syn_sub_init(&ctx->sub_fsm, &ctx->fsm, &chan_out_fsm);
}

static void listener_actuate_pwm_callback(const struct zbus_channel* chan)
{
    syn_sub_listen(&g_ctx.sub_actuators, chan, K_MSEC(1));
    syn_sub_listen(&g_ctx.sub_fsm, chan, K_MSEC(1));
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuate_pwm_callback);
ZBUS_CHAN_ADD_OBS(chan_out_actuators, listener_actuator_pwm, 1);
ZBUS_CHAN_ADD_OBS(chan_out_fsm, listener_actuator_pwm, 1);

void pwm_update(const synapse_msgs_Fsm* fsm, const synapse_msgs_Actuators* actuators)
{
    bool armed = fsm->armed == synapse_msgs_Fsm_Armed_ARMED;

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
        int err = 0;
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

void actuator_pwm_entry_point(context* ctx)
{
    init(ctx);

    while (true) {
        RC(syn_sub_poll(&ctx->sub_actuators, K_MSEC(1000)),
            LOG_DBG("not receiving actuators"));

        // update pwm
        RC(syn_sub_lock(&ctx->sub_actuators, K_MSEC(1)), continue;);
        RC(syn_sub_lock(&ctx->sub_fsm, K_MSEC(1)), continue;);
        pwm_update(&ctx->fsm, &ctx->actuators);
        RC(syn_sub_unlock(&ctx->sub_actuators), continue;);
        RC(syn_sub_unlock(&ctx->sub_fsm), continue;);
    }
}

K_THREAD_DEFINE(actuator_pwm, MY_STACK_SIZE,
    actuator_pwm_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
