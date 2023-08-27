/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(status_led, CONFIG_CEREBRI_ACTUATE_STATUS_LED_LOG_LEVEL);

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

static void run()
{

    while (true) {
        k_msleep(1000);
    }
}

K_THREAD_DEFINE(status_led, MY_STACK_SIZE,
    run, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
