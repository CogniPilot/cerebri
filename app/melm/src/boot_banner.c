/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_CEREBRI_CORE_COMMON_BOOT_BANNER)
#include <stdio.h>
#endif

#include <cerebri/core/common.h>

LOG_MODULE_REGISTER(melm_boot_banner, CONFIG_CEREBRI_MELM_LOG_LEVEL);

static int melm_boot_banner_sys_init(void)
{
	LOG_INF("Cerebri MELM %d.%d.%d", CONFIG_CEREBRI_VERSION_MAJOR, CONFIG_CEREBRI_VERSION_MINOR,
		CONFIG_CEREBRI_VERSION_PATCH);
#if defined(CONFIG_CEREBRI_CORE_COMMON_BOOT_BANNER)
	printf("%s%s", banner_brain, banner_name);
#endif
	return 0;
};

SYS_INIT(melm_boot_banner_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
