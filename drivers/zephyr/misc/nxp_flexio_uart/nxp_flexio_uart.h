/*
 * Copyright 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NXP FlexIO UART Driver - Single-wire half-duplex UART using FlexIO
 *
 * This driver implements a software UART using the NXP FlexIO peripheral,
 * which can use almost any GPIO pin. It supports half-duplex single-wire
 * mode for protocols like AM32 ESC bootloader communication.
 */

#ifndef ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_UART_H_
#define ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_UART_H_

#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize FlexIO UART for transmit mode
 *
 * Configures the FlexIO shifter and timer for UART transmit.
 * Call this before transmitting data.
 *
 * @param dev FlexIO UART device
 * @param channel Channel index (for multi-channel configurations)
 * @return 0 on success, negative errno on failure
 */
int nxp_flexio_uart_tx_enable(const struct device *dev, uint8_t channel);

/**
 * @brief Initialize FlexIO UART for receive mode
 *
 * Configures the FlexIO shifter and timer for UART receive.
 * Call this before receiving data.
 *
 * @param dev FlexIO UART device
 * @param channel Channel index (for multi-channel configurations)
 * @return 0 on success, negative errno on failure
 */
int nxp_flexio_uart_rx_enable(const struct device *dev, uint8_t channel);

/**
 * @brief Transmit data over FlexIO UART
 *
 * Blocking transmit of data buffer. Must call nxp_flexio_uart_tx_enable()
 * first if not already in TX mode.
 *
 * @param dev FlexIO UART device
 * @param channel Channel index
 * @param data Pointer to data buffer
 * @param len Number of bytes to transmit
 * @return Number of bytes transmitted, or negative errno on failure
 */
int nxp_flexio_uart_write(const struct device *dev, uint8_t channel, const uint8_t *data,
			  size_t len);

/**
 * @brief Receive data from FlexIO UART
 *
 * Blocking receive into data buffer with timeout. Must call
 * nxp_flexio_uart_rx_enable() first if not already in RX mode.
 *
 * @param dev FlexIO UART device
 * @param channel Channel index
 * @param data Pointer to receive buffer
 * @param len Maximum number of bytes to receive
 * @param timeout_us Timeout in microseconds
 * @return Number of bytes received, or negative errno on failure/timeout
 */
int nxp_flexio_uart_read(const struct device *dev, uint8_t channel, uint8_t *data, size_t len,
			 uint32_t timeout_us);

/**
 * @brief Get number of configured channels
 *
 * @param dev FlexIO UART device
 * @return Number of channels
 */
uint8_t nxp_flexio_uart_channel_count(const struct device *dev);

/**
 * @brief Check if transmit is complete
 *
 * @param dev FlexIO UART device
 * @param channel Channel index
 * @return true if transmit complete, false if still transmitting
 */
bool nxp_flexio_uart_tx_complete(const struct device *dev, uint8_t channel);

/**
 * @brief Set pin to a specific level (HIGH or LOW)
 *
 * Disables UART mode and directly controls the pin level. This is useful
 * for protocols that require holding the pin at a specific level for
 * a period of time (e.g., reset pulses, bootloader entry).
 *
 * After using this function, call nxp_flexio_uart_tx_enable() or
 * nxp_flexio_uart_rx_enable() to return to UART mode.
 *
 * @param dev FlexIO UART device
 * @param channel Channel index
 * @param level true for HIGH, false for LOW
 * @return 0 on success, negative errno on failure
 */
int nxp_flexio_uart_set_pin(const struct device *dev, uint8_t channel, bool level);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_MISC_NXP_FLEXIO_UART_H_ */
