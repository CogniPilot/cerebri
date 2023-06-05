/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#include <synapse_zbus/channels.h>

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 4


static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));

//static const uint32_t servo_min_pulse = DT_PROP(DT_NODELABEL(servo), min_pulse);
//static const uint32_t servo_max_pulse = DT_PROP(DT_NODELABEL(servo), max_pulse);


#define STEP PWM_USEC(100)

static Actuators g_actuators = Actuators_init_zero;

static void listener_actuator_pwm_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_out_actuators) {
        g_actuators = *(Actuators*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_actuator_pwm, listener_actuator_pwm_callback);

void actuator_pwm_entry_point(void * p1, void * p2, void * p3) {

    if (!device_is_ready(servo.dev)) {
        printk("Error: PWM device %s is not ready\n", servo.dev->name);
        return;
    }

    while (true) {
        // turn angle
        uint32_t servo_steering = 1500 + g_actuators.normalized[0]*500;
		int ret = pwm_set_pulse_dt(&servo, servo_steering);
        if (ret < 0) {
            printk("Error: PWM device set pulse failed\n");
        }

        // TODO throttle
        //uint32_t servo_throttle = 1500 + g_actuators.normalized[0]*500;
		//int ret = pwm_set_pulse_dt(&servo1, pulse_width);
        //if (ret < 0) {
        //    printk("Error: PWM device set pulse failed\n");
        //}

        // sleep to set control rate at 50 Hz
        k_usleep(1e6/50);
    }
}

K_THREAD_DEFINE(actuator_pwm_thread, MY_STACK_SIZE,
    actuator_pwm_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
