/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief NXP FlexIO Dshot public API
 */

#ifndef ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_DSHOT_NXP_FLEXIO_DSHOT_H_
#define ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_DSHOT_NXP_FLEXIO_DSHOT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FlexIO Dshot driver public APIs
 * @defgroup flexio_dshot_interface FlexIO Dshot driver APIs
 * @ingroup misc_interfaces
 * @{
 */

void nxp_flexio_dshot_data_set(const struct device* dev, unsigned channel,
    uint16_t throttle, bool telemetry);

void nxp_flexio_dshot_trigger(const struct device* dev);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_DSHOT_NXP_FLEXIO_DSHOT_H_ */
