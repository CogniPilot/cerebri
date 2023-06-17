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
#include <zephyr/shell/shell.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

extern actuator_pwm_t actuator_pwms[];

static synapse_msgs_Actuators g_actuators = synapse_msgs_Actuators_init_zero;

static void listener_actuator_pwm_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(synapse_msgs_Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuator_pwm_callback);

void actuator_pwm_entry_point(const struct shell* sh)
{

    while (true) {

        for (int i = 0; i < CONFIG_ACTUATE_PWM_NUMBER; i++) {
            actuator_pwm_t pwm = actuator_pwms[i];
            if (pwm.max < pwm.center || pwm.min > pwm.center) {
                /*
                printf("actuator_pwm: config pwm_%d min, center, "
                                "max must monotonically increase\n",
                    i);
                    */
                continue;
            }
            uint16_t pulse = pwm.center;
            if (pwm.type == PWM_TYPE_NORMALIZED) {
                float input = g_actuators.normalized[pwm.index];
                if (input < -1 || input > 1) {
                    // printf("actuator_pwm: normalized input out of bounds\n");
                    continue;
                }
                if (input > 0) {
                    pulse += input * (pwm.max - pwm.center);
                } else {
                    pulse += input * (pwm.center - pwm.min);
                }
            } else if (pwm.type == PWM_TYPE_POSITION) {
                float input = g_actuators.position[pwm.index];
                uint16_t output = (uint16_t)((pwm.slope * input) + pwm.intercept);
                // printf("actuator_pwm: %s position index %d with input %f output %d\n", pwm.alias, pwm.index, input, output);
                if (output > pwm.max) {
                    pulse = pwm.max;
                    // printf("actuator_pwm: %s  position command saturated, requested %d > %d\n", pwm.alias, output, pwm.max);
                } else if (output < pwm.min) {
                    pulse = pwm.min;
                    // printf("actuator_pwm: %s  position command saturated, requested %d < %d\n", pwm.alias, output, pwm.min);
                } else {
                    pulse = output;
                }
            } else if (pwm.type == PWM_TYPE_VELOCITY) {
                float input = g_actuators.velocity[pwm.index];
                uint16_t output = (uint16_t)((pwm.slope * input) + pwm.intercept);
                // printf("actuator_pwm: %s  velocity index %d with input %f output %d\n", pwm.alias, pwm.index, input, output);
                if (output > pwm.max) {
                    pulse = pwm.max;
                    // printf("actuator_pwm: %s velocity command saturated, requested %d > %d\n", pwm.alias, output, pwm.max);
                } else if (output < pwm.min) {
                    pulse = pwm.min;
                    // printf("actuator_pwm: %s  velocity command saturated, requested %d < %d\n", pwm.alias, output, pwm.min);
                } else {
                    pulse = output;
                }
            }
            int err = 0;
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
            if (err) {
                // printf("actuator_pwm: failed to set pulse %d on %s (err %d)\n",
                //    pulse, pwm.alias, err);
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
