/*
 * Copyright 2024 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for NXP FlexIO UART driver
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(flexio_uart_test, LOG_LEVEL_DBG);

/* Include the driver header */
#include "nxp_flexio_uart.h"

/* Get FlexIO UART device from device tree */
#if DT_HAS_COMPAT_STATUS_OKAY(nxp_flexio_uart)
#define FLEXIO_UART_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(nxp_flexio_uart)
static const struct device *flexio_uart_dev = DEVICE_DT_GET(FLEXIO_UART_NODE);
#else
static const struct device *flexio_uart_dev = NULL;
#endif

/**
 * Test that the FlexIO UART device is ready
 */
ZTEST(flexio_uart, test_device_ready)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	zassert_true(device_is_ready(flexio_uart_dev), "FlexIO UART device is not ready");

	LOG_INF("FlexIO UART device is ready");
}

/**
 * Test channel count
 */
ZTEST(flexio_uart, test_channel_count)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	uint8_t count = nxp_flexio_uart_channel_count(flexio_uart_dev);
	zassert_true(count > 0, "Expected at least one channel, got %d", count);

	LOG_INF("FlexIO UART has %d channels", count);
}

/**
 * Test TX mode enable
 */
ZTEST(flexio_uart, test_tx_enable)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	int ret = nxp_flexio_uart_tx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable TX mode: %d", ret);

	LOG_INF("TX mode enabled successfully");
}

/**
 * Test RX mode enable
 */
ZTEST(flexio_uart, test_rx_enable)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	int ret = nxp_flexio_uart_rx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable RX mode: %d", ret);

	LOG_INF("RX mode enabled successfully");
}

/**
 * Test invalid channel handling
 */
ZTEST(flexio_uart, test_invalid_channel)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	/* Try to use an invalid channel number */
	int ret = nxp_flexio_uart_tx_enable(flexio_uart_dev, 255);
	zassert_equal(ret, -EINVAL, "Expected -EINVAL for invalid channel, got %d", ret);

	LOG_INF("Invalid channel handling works correctly");
}

/**
 * Test TX write (basic - no actual hardware verification)
 *
 * This test verifies the write function completes without error.
 * Actual data verification requires an oscilloscope or loopback setup.
 */
ZTEST(flexio_uart, test_tx_write)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	const uint8_t test_data[] = {0x55, 0xAA, 0x00, 0xFF};
	int ret;

	ret = nxp_flexio_uart_tx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable TX: %d", ret);

	ret = nxp_flexio_uart_write(flexio_uart_dev, 0, test_data, sizeof(test_data));
	zassert_equal(ret, sizeof(test_data), "Write failed: %d", ret);

	LOG_INF("TX write completed, sent %d bytes", ret);
}

/**
 * Test RX read with timeout
 *
 * This test verifies the read function handles timeout correctly
 * when no data is available.
 */
ZTEST(flexio_uart, test_rx_read_timeout)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	uint8_t rx_buf[4];
	int ret;

	ret = nxp_flexio_uart_rx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable RX: %d", ret);

	/* Try to read with a short timeout - should return 0 bytes */
	ret = nxp_flexio_uart_read(flexio_uart_dev, 0, rx_buf, sizeof(rx_buf), 1000);
	zassert_true(ret >= 0, "Read returned error: %d", ret);

	LOG_INF("RX read with timeout completed, received %d bytes", ret);
}

/**
 * Test mode switching (TX -> RX -> TX)
 */
ZTEST(flexio_uart, test_mode_switching)
{
	if (flexio_uart_dev == NULL) {
		ztest_test_skip();
		return;
	}

	int ret;

	/* Switch to TX */
	ret = nxp_flexio_uart_tx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable TX: %d", ret);

	/* Switch to RX */
	ret = nxp_flexio_uart_rx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to enable RX: %d", ret);

	/* Switch back to TX */
	ret = nxp_flexio_uart_tx_enable(flexio_uart_dev, 0);
	zassert_equal(ret, 0, "Failed to re-enable TX: %d", ret);

	LOG_INF("Mode switching works correctly");
}

ZTEST_SUITE(flexio_uart, NULL, NULL, NULL, NULL, NULL);
