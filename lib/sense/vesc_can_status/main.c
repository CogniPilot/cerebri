/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(sense_vesc_can_status, CONFIG_CEREBRI_SENSE_VESC_CAN_STATUS_LOG_LEVEL);

void vesc_can_status_entry_point(void* p0, void* p1, void* p2)
{
    LOG_INF("init");
    ARG_UNUSED(p0);
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    return;
}

K_THREAD_DEFINE(sense_vesc_can_status, MY_STACK_SIZE,
    vesc_can_status_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
