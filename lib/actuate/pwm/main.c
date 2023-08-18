/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuator_pwm.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <synapse/zbus/channels.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(actuate_pwm, CONFIG_ACTUATE_PWM_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

extern actuator_pwm_t g_actuator_pwms[];

static synapse_msgs_Actuators g_actuators = synapse_msgs_Actuators_init_default;

static void listener_actuator_pwm_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(synapse_msgs_Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuator_pwm_callback);

void actuator_pwm_entry_point()
{

    while (true) {

        for (int i = 0; i < CONFIG_ACTUATE_PWM_NUMBER; i++) {
            actuator_pwm_t pwm = g_actuator_pwms[i];
            if (pwm.max < pwm.center || pwm.min > pwm.center) {
                LOG_ERR("config pwm_%d min, center, "
                        "max must monotonically increase",
                    i);
                continue;
            }
            uint32_t pulse = pwm.center;
            if (pwm.type == PWM_TYPE_NORMALIZED) {
                float input = g_actuators.normalized[pwm.index];
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
                float input = g_actuators.position[pwm.index];
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
                float input = g_actuators.velocity[pwm.index];
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

        // sleep to set control rate at 50 Hz
        k_usleep(1e6 / 50);
    }
}

K_THREAD_DEFINE(actuator_pwm_thread, MY_STACK_SIZE,
    actuator_pwm_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
