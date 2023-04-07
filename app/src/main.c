/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "app_version.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static volatile int keepRunning = 1;

void intHandler(int dummy)
{
    (void)dummy;
    keepRunning = 0;
    printf("sigint caught\n");
    exit(0);
}

void main(void)
{
    printk("Cerebri %s\n", APP_VERSION_STR);
    signal(SIGINT, intHandler);
    while (keepRunning) {
        k_msleep(5000);
        int64_t uptime = k_uptime_get() / 1e3;
        printf("uptime: %ld sec\n", uptime);
    }
}

// vi: ts=4 sw=4 et
