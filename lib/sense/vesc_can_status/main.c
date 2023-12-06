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

void vesc_can_entry_point(void* p0, void* p1, void* p2)
{
    ARG_UNUSED(p0);
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    return;
}

K_THREAD_DEFINE(can_vesc, MY_STACK_SIZE,
    vesc_can_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
