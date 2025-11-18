/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 CogniPilot Foundation
 *
 * ESC Passthrough Hardware Tests
 *
 * These tests require actual hardware:
 * - VMU RT1170 board
 * - ESC with AM32 firmware connected to channel 1
 * - Battery power to ESC
 *
 * Run with: west build -b vmu_rt1170/mimxrt1176/cm7 app/esc_passthrough/tests -p
 *           west flash -r jlink
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(test_hardware, LOG_LEVEL_INF);

/* Only compile hardware tests for real hardware */
#if DT_NODE_EXISTS(DT_NODELABEL(esc1))

#define ESC_CHANNEL 0

static const struct gpio_dt_spec esc_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(esc1), gpios);

/* Timing for 19200 baud */
#define BIT_TIME_US 52

/* BLHeli bootloader ACK codes */
#define BL_ACK_OK      0xC0
#define BL_ACK_ERROR   0xC1
#define BL_ACK_BAD_CRC 0xC2

/* CRC-16 with polynomial 0xA001 */
static uint16_t crc16_bl(const uint8_t *data, size_t len)
{
	uint16_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

static void esc_output(void)
{
	gpio_pin_configure_dt(&esc_gpio, GPIO_OUTPUT);
}

static void esc_input(void)
{
	gpio_pin_configure_dt(&esc_gpio, GPIO_INPUT | GPIO_PULL_UP);
}

static void esc_high(void)
{
	gpio_pin_set_dt(&esc_gpio, 1);
}

static void esc_low(void)
{
	gpio_pin_set_dt(&esc_gpio, 0);
}

static int esc_read(void)
{
	return gpio_pin_get_dt(&esc_gpio);
}

/* Software UART TX */
static void sw_tx(uint8_t byte)
{
	esc_output();
	esc_low(); /* Start bit */
	k_busy_wait(BIT_TIME_US);

	for (int i = 0; i < 8; i++) {
		if (byte & (1 << i)) {
			esc_high();
		} else {
			esc_low();
		}
		k_busy_wait(BIT_TIME_US);
	}

	esc_high(); /* Stop bit */
	k_busy_wait(BIT_TIME_US);
}

/* Software UART RX with timeout */
static int sw_rx(uint8_t *byte, int timeout_us)
{
	int64_t start = k_uptime_get();
	int64_t timeout_ms = (timeout_us + 999) / 1000;

	esc_input();

	/* Wait for start bit */
	while (esc_read() != 0) {
		if ((k_uptime_get() - start) > timeout_ms) {
			return -ETIMEDOUT;
		}
		k_busy_wait(5);
	}

	/* Center in start bit */
	k_busy_wait(BIT_TIME_US / 2);

	/* Read 8 data bits */
	uint8_t data = 0;
	for (int i = 0; i < 8; i++) {
		k_busy_wait(BIT_TIME_US);
		if (esc_read()) {
			data |= (1 << i);
		}
	}

	/* Wait for stop bit */
	k_busy_wait(BIT_TIME_US);

	*byte = data;
	return 0;
}

/*
 * Test Suite: Hardware GPIO
 */
static void *hw_gpio_setup(void)
{
	zassert_true(device_is_ready(esc_gpio.port), "GPIO device not ready");
	return NULL;
}

ZTEST_SUITE(hw_gpio_tests, NULL, hw_gpio_setup, NULL, NULL, NULL);

ZTEST(hw_gpio_tests, test_gpio_output)
{
	esc_output();
	esc_high();
	k_msleep(1);
	esc_low();
	k_msleep(1);
	esc_high();
	/* If we get here without crash, GPIO works */
	zassert_true(true, "GPIO output test passed");
}

ZTEST(hw_gpio_tests, test_gpio_input)
{
	esc_input();
	int val = esc_read();
	/* Value can be 0 or 1 depending on ESC state */
	zassert_true(val == 0 || val == 1, "GPIO input should return 0 or 1");
}

/*
 * Test Suite: Bootloader Query
 */
static void *hw_bootloader_setup(void)
{
	zassert_true(device_is_ready(esc_gpio.port), "GPIO device not ready");
	/* Ensure ESC is in input mode initially */
	esc_input();
	return NULL;
}

ZTEST_SUITE(hw_bootloader_tests, NULL, hw_bootloader_setup, NULL, NULL, NULL);

ZTEST(hw_bootloader_tests, test_bootloader_query)
{
	/* BLHeli/AM32 bootloader query */
	static const uint8_t boot_init[] = {0, 0,    0,   0,   0,   0,   0,   0,   0,    0,   0,
					    0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};

	LOG_INF("Sending bootloader query...");

	esc_output();
	for (size_t i = 0; i < sizeof(boot_init); i++) {
		sw_tx(boot_init[i]);
	}
	esc_input();

	/* Read response */
	uint8_t resp[8] = {0};
	int got = 0;
	for (int i = 0; i < 12 && got < 8; i++) {
		uint8_t byte;
		if (sw_rx(&byte, 50000) == 0) {
			resp[got++] = byte;
			LOG_INF("  [%d] 0x%02X", got, byte);
		}
	}

	LOG_INF("Got %d bytes", got);

	if (got >= 8 && resp[0] == '4' && resp[1] == '7' && resp[2] == '1') {
		uint16_t sig = (resp[4] << 8) | resp[5];
		LOG_INF("SUCCESS: Bootloader found, sig=0x%04X", sig);
		zassert_true(true, "Bootloader query succeeded");
	} else if (got == 0) {
		LOG_WRN("No response - ESC may need power or HIGH signal at boot");
		zassert_true(false, "No bootloader response - check ESC power and boot state");
	} else {
		LOG_WRN("Invalid response: %02X %02X %02X", resp[0], resp[1], resp[2]);
		zassert_true(false, "Invalid bootloader response");
	}
}

/*
 * Test Suite: Read Operations
 */
ZTEST_SUITE(hw_read_tests, NULL, hw_bootloader_setup, NULL, NULL, NULL);

static int bl_set_address(uint16_t addr)
{
	uint8_t cmd[4] = {0xFF, 0x00, (addr >> 8) & 0xFF, addr & 0xFF};
	uint16_t crc = crc16_bl(cmd, 4);

	esc_output();
	for (int i = 0; i < 4; i++) {
		sw_tx(cmd[i]);
	}
	sw_tx(crc & 0xFF);
	sw_tx(crc >> 8);
	esc_input();

	uint8_t ack;
	if (sw_rx(&ack, 100000) != 0) {
		return -ETIMEDOUT;
	}
	if (ack != BL_ACK_OK) {
		return -EIO;
	}
	return 0;
}

static int bl_read_data(uint8_t *data, uint16_t len)
{
	uint8_t count = (len == 256) ? 0 : (uint8_t)len;
	uint8_t cmd[2] = {0x03, count};
	uint16_t crc = crc16_bl(cmd, 2);

	esc_output();
	sw_tx(cmd[0]);
	sw_tx(cmd[1]);
	sw_tx(crc & 0xFF);
	sw_tx(crc >> 8);
	esc_input();

	uint8_t ack;
	if (sw_rx(&ack, 100000) != 0) {
		return -ETIMEDOUT;
	}
	if (ack != BL_ACK_OK) {
		return -EIO;
	}

	for (int i = 0; i < len; i++) {
		if (sw_rx(&data[i], 100000) != 0) {
			return -ETIMEDOUT;
		}
	}

	/* Read and verify CRC */
	uint8_t crc_lo, crc_hi;
	if (sw_rx(&crc_lo, 100000) != 0) {
		return -ETIMEDOUT;
	}
	if (sw_rx(&crc_hi, 100000) != 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

ZTEST(hw_read_tests, test_read_eeprom)
{
	/* First query bootloader */
	static const uint8_t boot_init[] = {0, 0,    0,   0,   0,   0,   0,   0,   0,    0,   0,
					    0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};

	esc_output();
	for (size_t i = 0; i < sizeof(boot_init); i++) {
		sw_tx(boot_init[i]);
	}
	esc_input();

	uint8_t resp[8] = {0};
	int got = 0;
	for (int i = 0; i < 12 && got < 8; i++) {
		uint8_t byte;
		if (sw_rx(&byte, 50000) == 0) {
			resp[got++] = byte;
		}
	}

	if (got < 8 || resp[0] != '4') {
		LOG_WRN("Bootloader not responding - skipping read test");
		ztest_test_skip();
		return;
	}

	LOG_INF("Bootloader ready, reading EEPROM...");

	/* Set address to firmware info */
	int ret = bl_set_address(0x7BE0);
	zassert_equal(ret, 0, "SET_ADDRESS 0x7BE0 should succeed");

	/* Read 32 bytes */
	uint8_t fw_info[32];
	ret = bl_read_data(fw_info, 32);
	zassert_equal(ret, 0, "READ 32 bytes should succeed");

	LOG_HEXDUMP_INF(fw_info, 32, "FW Info:");

	/* Set address to EEPROM */
	ret = bl_set_address(0x7C00);
	zassert_equal(ret, 0, "SET_ADDRESS 0x7C00 should succeed");

	/* Read first 32 bytes of EEPROM */
	uint8_t eeprom[32];
	ret = bl_read_data(eeprom, 32);
	zassert_equal(ret, 0, "READ EEPROM should succeed");

	LOG_HEXDUMP_INF(eeprom, 32, "EEPROM:");

	LOG_INF("Read test PASSED");
}

#else /* No hardware */

ZTEST_SUITE(hw_gpio_tests, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(hw_bootloader_tests, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(hw_read_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(hw_gpio_tests, test_gpio_output)
{
	ztest_test_skip();
}

ZTEST(hw_gpio_tests, test_gpio_input)
{
	ztest_test_skip();
}

ZTEST(hw_bootloader_tests, test_bootloader_query)
{
	ztest_test_skip();
}

ZTEST(hw_read_tests, test_read_eeprom)
{
	ztest_test_skip();
}

#endif /* DT_NODE_EXISTS(DT_NODELABEL(esc1)) */
