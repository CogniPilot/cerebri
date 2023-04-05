/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include "app_version.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
    printf("sigint caught\n");
    exit(0);
}

void main(void)
{
    printk("Cerebri %s\n", APP_VERSION_STR);
    signal(SIGINT, intHandler);
    while(keepRunning) {
        k_msleep(5000);
    }
}
