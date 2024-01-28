/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <cerebri/core/common.h>

LOG_MODULE_REGISTER(rdd2_main, CONFIG_CEREBRI_RDD2_LOG_LEVEL);

int main(void)
{
    LOG_INF("init");
#if defined(CONFIG_CEREBRI_CORE_COMMON_BOOT_BANNER)
    printf("%s%s", banner_brain, banner_name);
#endif
    LOG_INF("Cerebri RDD2 %d.%d.%d", CONFIG_CEREBRI_VERSION_MAJOR, CONFIG_CEREBRI_VERSION_MINOR, CONFIG_CEREBRI_VERSION_PATCH);
    return 0;
}

// vi: ts=4 sw=4 et
