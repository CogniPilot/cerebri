/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <synapse_zbus/channels.h>
#include <zephyr/drivers/pwm.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

const char* pwm_steering_name = "aux1";
const char* pwm_throttle_name = "aux2";
static const struct pwm_dt_spec pwm_steering = PWM_DT_SPEC_GET(
    DT_CHILD(DT_NODELABEL(pwm_shell), aux1));
static const struct pwm_dt_spec pwm_throttle = PWM_DT_SPEC_GET(
    DT_CHILD(DT_NODELABEL(pwm_shell), aux2));

static Actuators g_actuators = Actuators_init_zero;

static void listener_actuator_pwm_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuator_pwm_callback);

void actuator_pwm_entry_point(const struct shell *sh) {

    while (true) {
        // turn angle
        uint32_t servo_steering = 1500 + g_actuators.normalized[0]*500;
        uint32_t servo_throttle = 1500 + g_actuators.normalized[1]*500;
        int err = 0;
		err = pwm_set_pulse_dt(&pwm_steering, PWM_USEC(servo_steering));
        if (err) {
            shell_print(sh, "Failed to set pwm_steering on %s (err %d)",
                    pwm_steering_name, err);
        }

        err = pwm_set_pulse_dt(&pwm_throttle, PWM_USEC(servo_throttle));
        if (err) {
            shell_print(sh, "Failed to set pwm_throttle on %s (err %d)",
                    pwm_throttle_name, err);
        }

        // sleep to set control rate at 50 Hz
        k_usleep(1e6/50);
    }
}

K_THREAD_DEFINE(actuator_pwm_thread, MY_STACK_SIZE,
    actuator_pwm_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
