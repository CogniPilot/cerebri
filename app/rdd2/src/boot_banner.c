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
#include <cerebri/core/log_utils.h>

CEREBRI_NODE_LOG_INIT(rdd2_boot_banner, LOG_LEVEL_WRN);

static int rdd2_boot_banner_sys_init(void)
{
	LOG_INF("Cerebri RDD2 %d.%d.%d", CONFIG_CEREBRI_VERSION_MAJOR, CONFIG_CEREBRI_VERSION_MINOR,
		CONFIG_CEREBRI_VERSION_PATCH);
#if defined(CONFIG_CEREBRI_CORE_COMMON_BOOT_BANNER)
	printf("%s%s", banner_brain, banner_name);
#endif
	return 0;
};

SYS_INIT(rdd2_boot_banner_sys_init, APPLICATION, 0);

// vi: ts=4 sw=4 et
