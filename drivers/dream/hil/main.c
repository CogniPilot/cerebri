/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0 */

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   4

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dream_hil, CONFIG_CEREBRI_DREAM_HIL_LOG_LEVEL);

static void zephyr_hil_entry_point(void)
{
	LOG_WRN("running HIL");
}

// zephyr threads
K_THREAD_DEFINE(zephyr_hil, MY_STACK_SIZE, zephyr_hil_entry_point, NULL, NULL, NULL, MY_PRIORITY, 0,
		0);

// vi: ts=4 sw=4 et
