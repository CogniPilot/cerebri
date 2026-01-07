/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_MSC_H_
#define USB_MSC_H_

#include <stdbool.h>

/**
 * @brief Check if USB is physically connected (VBUS detected)
 * @return true if USB cable is connected, false otherwise
 */
bool usb_msc_is_connected(void);

/**
 * @brief Check if USB MSC mode is currently enabled
 * @return true if USB MSC is enabled, false otherwise
 */
bool usb_msc_is_enabled(void);

/**
 * @brief Enable USB MSC mode
 * @return 0 on success, -EALREADY if already enabled, negative errno on error
 */
int usb_msc_enable(void);

/**
 * @brief Disable USB MSC mode
 * @return 0 on success, -EALREADY if already disabled, negative errno on error
 */
int usb_msc_disable(void);

#endif /* USB_MSC_H_ */
