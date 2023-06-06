/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <synapse_zbus/channels.h>
#include <zephyr/drivers/pwm.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

static Actuators g_actuators = Actuators_init_zero;

static void listener_actuator_pwm_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuator_pwm_callback);

void actuator_pwm_entry_point(void * p1, void * p2, void * p3) {
    const struct device *devpwm0;
    devpwm0 = device_get_binding("pwm0");
    if (!devpwm0) {
        printf("PWM device pwm0 not found\n");
        return;
    }

    const struct device *devpwm1;
    devpwm1 = device_get_binding("pwm1");
    if (!devpwm1) {
        printf("PWM device pwm1 not found\n");
        return;
    }

    while (true) {
        // turn angle
        uint32_t servo_steering = 1500 + g_actuators.normalized[0]*500;
        uint32_t servo_throttle = 1500 + g_actuators.normalized[1]*500;
        int err = 0;
		err = pwm_set(devpwm0, 0, PWM_USEC(2000), PWM_USEC(servo_steering), 0);
        if (err) {
            printf("Failed to set PWM pwm0 (err %d)\n",
                    err);
        }

        err = pwm_set(devpwm1, 0, PWM_USEC(2000), PWM_USEC(servo_throttle), 0);
        if (err) {
            printf("Failed to set PWM pwm1 (err %d)\n",
                    err);
        }

        // sleep to set control rate at 50 Hz
        k_usleep(1e6/50);
    }
}

K_THREAD_DEFINE(actuator_pwm_thread, MY_STACK_SIZE,
    actuator_pwm_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
