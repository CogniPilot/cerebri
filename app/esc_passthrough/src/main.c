/*
 * Copyright (c) 2025 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESC Passthrough - MSP + 4-Way Interface for AM32 ESC Configuration
 *
 * This module implements passthrough communication between USB (via MSP protocol)
 * and AM32 ESC bootloaders (via single-wire serial at 19200 baud). It enables
 * configuration tools like am32.ca to read/write ESC settings through the VMU.
 *
 * Supported configurators:
 *   - https://am32.ca (AM32 Configurator)
 *   - https://esc-configurator.com
 *
 * Protocol stack:
 *   USB CDC-ACM -> MSP -> 4-Way Interface -> AM32 Bootloader (single-wire)
 *
 * Key protocol references:
 *   - AM32 Bootloader: https://github.com/am32-firmware/AM32-Bootloader
 *   - AM32 Configurator: https://github.com/am32-firmware/am32-configurator
 *
 * See tests/test_protocol.c for detailed protocol documentation.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(esc_passthrough, LOG_LEVEL_INF);

/* MSP commands - minimal set for ESC configurator */
#define MSP_API_VERSION   1
#define MSP_FC_VARIANT    2
#define MSP_FC_VERSION    3
#define MSP_BOARD_INFO    4
#define MSP_BUILD_INFO    5
#define MSP_MOTOR         104
#define MSP_BATTERY_STATE 130
#define MSP_MOTOR_CONFIG  131
#define MSP_SET_4WAY_IF   245

/* 4-way interface commands */
#define FOURWAY_START_MARKER          0x2F /* Request start marker (response uses 0x2E) */
#define FOURWAY_INTERFACE_TEST_ALIVE  0x30
#define FOURWAY_PROTOCOL_GET_VERSION  0x31
#define FOURWAY_INTERFACE_GET_NAME    0x32
#define FOURWAY_INTERFACE_GET_VERSION 0x33
#define FOURWAY_INTERFACE_EXIT        0x34
#define FOURWAY_DEVICE_RESET          0x35
#define FOURWAY_DEVICE_INIT_FLASH     0x37
#define FOURWAY_DEVICE_ERASE_ALL      0x38
#define FOURWAY_DEVICE_PAGE_ERASE     0x39
#define FOURWAY_DEVICE_READ           0x3A
#define FOURWAY_DEVICE_WRITE          0x3B
#define FOURWAY_DEVICE_READ_EEPROM    0x3D
#define FOURWAY_DEVICE_WRITE_EEPROM   0x3E
#define FOURWAY_INTERFACE_SET_MODE    0x3F

/* Response codes */
#define ACK_OK              0x00
#define ACK_UNKNOWN_ERROR   0x01
#define ACK_INVALID_CMD     0x02
#define ACK_INVALID_CRC     0x03
#define ACK_VERIFY_ERROR    0x04
#define ACK_INVALID_CHANNEL 0x08
#define ACK_GENERAL_ERROR   0x0F

/* ESC GPIO - count channels from device tree */
#define ESC_GPIOS_NODE DT_NODELABEL(esc_gpios)

/* Count ESC channels by checking which esc nodes exist */
#define ESC1_EXISTS DT_NODE_EXISTS(DT_NODELABEL(esc1))
#define ESC2_EXISTS DT_NODE_EXISTS(DT_NODELABEL(esc2))
#define ESC3_EXISTS DT_NODE_EXISTS(DT_NODELABEL(esc3))
#define ESC4_EXISTS DT_NODE_EXISTS(DT_NODELABEL(esc4))

#define NUM_ESC_CHANNELS (ESC1_EXISTS + ESC2_EXISTS + ESC3_EXISTS + ESC4_EXISTS)

#if NUM_ESC_CHANNELS == 0
#error "No ESC nodes defined in device tree (need esc1, esc2, etc.)"
#endif

static const struct gpio_dt_spec esc_gpios[NUM_ESC_CHANNELS] = {
#if ESC1_EXISTS
	GPIO_DT_SPEC_GET(DT_NODELABEL(esc1), gpios),
#endif
#if ESC2_EXISTS
	GPIO_DT_SPEC_GET(DT_NODELABEL(esc2), gpios),
#endif
#if ESC3_EXISTS
	GPIO_DT_SPEC_GET(DT_NODELABEL(esc3), gpios),
#endif
#if ESC4_EXISTS
	GPIO_DT_SPEC_GET(DT_NODELABEL(esc4), gpios),
#endif
};

static const struct device *const usb_uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

/* Pinctrl for ESC GPIOs - apply GPIO mux mode */
#if DT_NODE_HAS_PROP(DT_NODELABEL(esc_gpios), pinctrl_0)
PINCTRL_DT_DEFINE(DT_NODELABEL(esc_gpios));
#define HAS_ESC_PINCTRL 1
#else
#define HAS_ESC_PINCTRL 0
#endif

/* State */
static uint8_t current_esc = 0;
static bool fourway_mode = false;
static bool eeprom_erased[NUM_ESC_CHANNELS]; /* Track if EEPROM erased per ESC */

/* Buffers */
#define RX_BUF_SIZE 512
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile int rx_len = 0;

/*
 * Single-wire timing for AM32 bootloader protocol.
 * BIT_TIME_US is FIXED at 52 microseconds (19200 baud) and MUST NOT be changed.
 * This value is critical for protocol compatibility with AM32 ESCs.
 * Changing this value will break communication with the bootloader.
 */
#define BIT_TIME_US 52
/* Compile-time check: BIT_TIME_US must be 52 (19200 baud) */
#if BIT_TIME_US != 52
#error "BIT_TIME_US must be 52 (19200 baud) for AM32 bootloader protocol compatibility"
#endif

static inline void delay_us(uint32_t us)
{
	k_busy_wait(us);
}

static void esc_output(uint8_t esc)
{
	if (esc < NUM_ESC_CHANNELS) {
		gpio_pin_configure_dt(&esc_gpios[esc], GPIO_OUTPUT);
	}
}

static void esc_input(uint8_t esc)
{
	/* Use pull-up to keep line HIGH when idle (UART idle state) */
	if (esc < NUM_ESC_CHANNELS) {
		gpio_pin_configure_dt(&esc_gpios[esc], GPIO_INPUT | GPIO_PULL_UP);
	}
}

static void esc_high(uint8_t esc)
{
	if (esc < NUM_ESC_CHANNELS) {
		gpio_pin_set_dt(&esc_gpios[esc], 1);
	}
}

static void esc_low(uint8_t esc)
{
	if (esc < NUM_ESC_CHANNELS) {
		gpio_pin_set_dt(&esc_gpios[esc], 0);
	}
}

static int esc_read(uint8_t esc)
{
	return (esc < NUM_ESC_CHANNELS) ? gpio_pin_get_dt(&esc_gpios[esc]) : 0;
}

/* Single-wire TX byte */
static void sw_tx(uint8_t esc, uint8_t byte)
{
	esc_output(esc);
	esc_low(esc);
	delay_us(BIT_TIME_US);
	for (int i = 0; i < 8; i++) {
		if (byte & (1 << i)) {
			esc_high(esc);
		} else {
			esc_low(esc);
		}
		delay_us(BIT_TIME_US);
	}
	esc_high(esc);
	delay_us(BIT_TIME_US);
}

/* Single-wire RX byte with timeout */
static int sw_rx(uint8_t esc, uint8_t *byte, uint32_t timeout_us)
{
	uint32_t start = k_uptime_get_32();
	uint8_t data = 0;

	esc_input(esc);
	while (esc_read(esc)) {
		uint32_t elapsed_ms = k_uptime_get_32() - start;
		if ((uint64_t)elapsed_ms * 1000 > timeout_us) {
			return -ETIMEDOUT;
		}
	}
	delay_us(BIT_TIME_US + BIT_TIME_US / 2);
	for (int i = 0; i < 8; i++) {
		if (esc_read(esc)) {
			data |= (1 << i);
		}
		delay_us(BIT_TIME_US);
	}
	*byte = data;
	return 0;
}

/* CRC16 for 4-way protocol (polynomial 0x1021, MSB first) */
static uint16_t crc16(const uint8_t *data, int len)
{
	uint16_t crc = 0;
	for (int i = 0; i < len; i++) {
		crc ^= data[i] << 8;
		for (int j = 0; j < 8; j++) {
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
		}
	}
	return crc;
}

/* CRC16 for AM32 bootloader protocol (polynomial 0xA001, LSB first / reflected) */
static uint16_t crc16_bl(const uint8_t *data, int len)
{
	uint16_t crc = 0;
	for (int i = 0; i < len; i++) {
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

/* AM32 bootloader commands */
#define BL_CMD_RUN          0x00
#define BL_CMD_PROG_FLASH   0x01
#define BL_CMD_ERASE_FLASH  0x02
#define BL_CMD_READ_FLASH   0x03
#define BL_CMD_VERIFY_FLASH 0x04 /* Not used in AM32 */
#define BL_CMD_SET_BUFFER   0xFE
#define BL_CMD_SET_ADDRESS  0xFF
#define BL_CMD_KEEP_ALIVE   0xFD

#define BL_ACK_OK      0x30 /* '0' = Success */
#define BL_ACK_BAD_CMD 0xC1 /* Bad command / operation failed */
#define BL_ACK_BAD_CRC 0xC2 /* CRC mismatch */

/* Send a bootloader command with CRC and wait for ACK */
static int bl_send_cmd(uint8_t esc, const uint8_t *cmd, int cmd_len, uint8_t *ack_out)
{
	/* Calculate CRC on command bytes */
	uint16_t crc = crc16_bl(cmd, cmd_len);

	/* Send command bytes */
	for (int i = 0; i < cmd_len; i++) {
		sw_tx(esc, cmd[i]);
	}
	/* Send CRC (low byte first) */
	sw_tx(esc, crc & 0xFF);
	sw_tx(esc, crc >> 8);

	/* Turnaround delay - give bootloader time to process before we listen */
	delay_us(200);

	/* Wait for ACK */
	uint8_t ack;
	if (sw_rx(esc, &ack, 100000) != 0) {
		LOG_WRN("BL: no ACK after cmd 0x%02X", cmd[0]);
		return -ETIMEDOUT;
	}
	if (ack_out) {
		*ack_out = ack;
	}

	if (ack == BL_ACK_OK) {
		return 0;
	} else if (ack == BL_ACK_BAD_CRC) {
		LOG_WRN("BL: BAD_CRC for cmd 0x%02X", cmd[0]);
		return -EINVAL;
	} else {
		LOG_WRN("BL: unexpected ACK 0x%02X for cmd 0x%02X", ack, cmd[0]);
		return -EIO;
	}
}

/* Set bootloader address before read/write */
static int bl_set_address(uint8_t esc, uint16_t addr)
{
	uint8_t cmd[4] = {BL_CMD_SET_ADDRESS, 0x00, addr >> 8, addr & 0xFF};
	LOG_DBG("BL: SET_ADDRESS 0x%04X", addr);
	return bl_send_cmd(esc, cmd, 4, NULL);
}

/* Read data from bootloader at previously set address */
static int bl_read_data(uint8_t esc, uint8_t *data, uint16_t len)
{
	/* Send READ command: [0x03, size] + CRC */
	uint8_t cmd[2] = {BL_CMD_READ_FLASH, (len == 256) ? 0 : (uint8_t)len};
	uint16_t crc = crc16_bl(cmd, 2);

	sw_tx(esc, cmd[0]);
	sw_tx(esc, cmd[1]);
	sw_tx(esc, crc & 0xFF);
	sw_tx(esc, crc >> 8);

	/* Turnaround delay */
	delay_us(200);

	/* Read data bytes */
	for (int i = 0; i < len; i++) {
		if (sw_rx(esc, &data[i], 100000) != 0) {
			LOG_WRN("BL READ: timeout at byte %d/%d", i, len);
			return -ETIMEDOUT;
		}
	}

	/* Read CRC (low byte first) */
	uint8_t crc_lo, crc_hi;
	if (sw_rx(esc, &crc_lo, 50000) != 0 || sw_rx(esc, &crc_hi, 50000) != 0) {
		LOG_WRN("BL READ: CRC timeout");
		return -ETIMEDOUT;
	}

	/* Read ACK */
	uint8_t ack;
	if (sw_rx(esc, &ack, 50000) != 0) {
		LOG_WRN("BL READ: ACK timeout");
		return -ETIMEDOUT;
	}

	/* Verify CRC */
	uint16_t calc_crc = crc16_bl(data, len);
	uint16_t rx_crc = (crc_hi << 8) | crc_lo;
	if (calc_crc != rx_crc) {
		LOG_WRN("BL READ: CRC mismatch (calc=0x%04X rx=0x%04X)", calc_crc, rx_crc);
		return -EIO;
	}

	if (ack != BL_ACK_OK) {
		LOG_WRN("BL READ: bad ACK 0x%02X", ack);
		return -EIO;
	}

	LOG_DBG("BL READ: %d bytes OK", len);
	return 0;
}

/* Write data to bootloader at previously set address */
static int bl_write_data(uint8_t esc, const uint8_t *data, uint16_t len)
{
	/*
	 * AM32 bootloader write protocol (3 steps):
	 * Reference: https://github.com/am32-firmware/AM32-Bootloader/blob/master/bootloader/main.c
	 *
	 * 1. SET_BUFFER: [0xFE, 0x00, type, count] + CRC -> NO ACK!
	 *    - Sets payload_buffer_size and incoming_payload_no_command flag
	 *    - type=0x01 means 256 bytes, else count byte (rxBuffer[3]) is used
	 *    - IMPORTANT: Bootloader does NOT send ACK after SET_BUFFER!
	 *      It sets incoming_payload_no_command=1 and calls setReceive() to
	 *      immediately wait for data bytes.
	 *
	 * 2. DATA: [data bytes...] + CRC(data) -> ACK
	 *    - NO count prefix! Bootloader knows size from SET_BUFFER
	 *    - receiveBuffer() collects exactly payload_buffer_size + 2 (CRC) bytes
	 *    - CRC is validated on the raw data (not including SET_BUFFER cmd)
	 *
	 * 3. PROG_FLASH: [0x01, 0x00] + CRC -> ACK
	 *    - Writes payLoadBuffer to flash at current address
	 *    - Returns 0x30 (ACK_OK) on success, 0xC1 (BAD_CMD) on failure
	 *
	 * Key insight from official repo: SET_BUFFER handler does NOT call send_ACK(),
	 * it only sets state and waits for payload. Client must NOT wait for ACK!
	 */
	uint8_t ack;
	uint16_t crc;

	/* Step 1: SET_BUFFER - tell bootloader how much data to expect */
	uint8_t set_buf_cmd[4];
	set_buf_cmd[0] = BL_CMD_SET_BUFFER; /* 0xFE */
	set_buf_cmd[1] = 0x00;
	if (len == 256) {
		set_buf_cmd[2] = 0x01; /* Type: 256 bytes */
		set_buf_cmd[3] = 0x00;
	} else {
		set_buf_cmd[2] = 0x00; /* Type: use count */
		set_buf_cmd[3] = (uint8_t)len;
	}
	crc = crc16_bl(set_buf_cmd, 4);

	/* Minimize logging during write for speed - use LOG_DBG for verbose */
	LOG_DBG("BL WRITE: SET_BUFFER len=%d", len);

	sw_tx(esc, set_buf_cmd[0]);
	sw_tx(esc, set_buf_cmd[1]);
	sw_tx(esc, set_buf_cmd[2]);
	sw_tx(esc, set_buf_cmd[3]);
	sw_tx(esc, crc & 0xFF);
	sw_tx(esc, crc >> 8);

	/*
	 * NO ACK expected after SET_BUFFER!
	 * Per official AM32-Bootloader: the handler sets incoming_payload_no_command=1
	 * and immediately calls setReceive() to wait for data. No send_ACK() is called.
	 * Give bootloader time to prepare receive buffer.
	 */
	delay_us(500);

	/*
	 * Step 2: Send data + CRC
	 * Per official AM32-Bootloader decodeInput(): when incoming_payload_no_command=1,
	 * bootloader collects exactly payload_buffer_size bytes + 2 CRC bytes,
	 * validates CRC, stores in payLoadBuffer, then DOES send ACK (or BAD_CRC_ACK).
	 */
	uint16_t data_crc = crc16_bl(data, len);

	for (int i = 0; i < len; i++) {
		sw_tx(esc, data[i]);
	}
	sw_tx(esc, data_crc & 0xFF);
	sw_tx(esc, data_crc >> 8);

	/* Turnaround delay */
	delay_us(200);

	if (sw_rx(esc, &ack, 100000) != 0) {
		LOG_WRN("BL WRITE: no ACK for data");
		return -ETIMEDOUT;
	}
	if (ack != BL_ACK_OK) {
		LOG_WRN("BL WRITE: bad data ACK 0x%02X", ack);
		return -EIO;
	}
	LOG_DBG("BL WRITE: data ACK OK");

	/* Step 3: PROG_FLASH - write buffered data to flash */
	uint8_t prog_cmd[2] = {BL_CMD_PROG_FLASH, 0x00};
	crc = crc16_bl(prog_cmd, 2);

	LOG_DBG("BL WRITE: PROG_FLASH");

	sw_tx(esc, prog_cmd[0]);
	sw_tx(esc, prog_cmd[1]);
	sw_tx(esc, crc & 0xFF);
	sw_tx(esc, crc >> 8);

	/* Turnaround delay */
	delay_us(200);

	/* Wait for final ACK (flash programming can take time) */
	if (sw_rx(esc, &ack, 500000) != 0) {
		LOG_WRN("BL WRITE: no ACK for PROG_FLASH");
		return -ETIMEDOUT;
	}

	if (ack == BL_ACK_BAD_CMD) {
		LOG_ERR("BL WRITE: PROG_FLASH FAILED (0xC1)");
		return -EIO;
	} else if (ack == BL_ACK_BAD_CRC) {
		LOG_ERR("BL WRITE: CRC error (0xC2)");
		return -EIO;
	} else if (ack != BL_ACK_OK) {
		LOG_WRN("BL WRITE: unexpected PROG_FLASH ACK 0x%02X", ack);
		return -EIO;
	}

	LOG_DBG("BL WRITE: %d bytes PROG_FLASH OK", len);
	return 0;
}

/* Erase flash page at previously set address */
static int bl_erase_page(uint8_t esc)
{
	uint8_t cmd[2] = {BL_CMD_ERASE_FLASH, 0x00};
	LOG_DBG("BL: ERASE_PAGE");
	/* Erase can take a while, use longer timeout by sending manually */
	uint16_t crc = crc16_bl(cmd, 2);
	sw_tx(esc, cmd[0]);
	sw_tx(esc, cmd[1]);
	sw_tx(esc, crc & 0xFF);
	sw_tx(esc, crc >> 8);

	/* Turnaround delay */
	delay_us(200);

	/* Wait for ACK - erase can take up to 500ms */
	uint8_t ack;
	if (sw_rx(esc, &ack, 500000) != 0) {
		LOG_WRN("BL ERASE: no ACK");
		return -ETIMEDOUT;
	}

	if (ack == BL_ACK_OK) {
		LOG_DBG("BL ERASE: OK");
		return 0;
	} else if (ack == BL_ACK_BAD_CRC) {
		LOG_WRN("BL ERASE: BAD_CRC");
		return -EINVAL;
	} else {
		LOG_WRN("BL ERASE: unexpected ACK 0x%02X", ack);
		return -EIO;
	}
}

/* Send MSP response */
static void msp_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
	/* Skip USB ready check - it can cause issues with web serial */

	uint8_t cksum = len ^ cmd;
	uart_poll_out(usb_uart, '$');
	uart_poll_out(usb_uart, 'M');
	uart_poll_out(usb_uart, '>');
	uart_poll_out(usb_uart, len);
	uart_poll_out(usb_uart, cmd);
	for (int i = 0; i < len; i++) {
		uart_poll_out(usb_uart, data[i]);
		cksum ^= data[i];
	}
	uart_poll_out(usb_uart, cksum);
	LOG_DBG("MSP resp cmd=%d len=%d", cmd, len);
}

/* Send 4-way response - AM32 format (send directly like MSP) */
/* Format: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack] [crc_hi] [crc_lo] */
/*
 * IMPORTANT: param_count=0 means 256 bytes in the AM32 configurator!
 * For responses with no data, we must include a dummy byte to avoid this.
 * Reference: am32-configurator/src/communication/four_way.ts parseMessage()
 */
static void fourway_send_ex(uint8_t cmd, uint16_t addr, uint8_t ack, const uint8_t *data,
			    uint16_t len)
{
	uint8_t buf[270];
	int idx = 0;

	/* Build packet for CRC calculation */
	buf[idx++] = 0x2E;
	buf[idx++] = cmd;
	buf[idx++] = addr >> 8;
	buf[idx++] = addr & 0xFF;

	/*
	 * Handle param_count: 0 means 256 in configurator, so for empty responses
	 * we include a dummy 1-byte payload to avoid the "0=256" interpretation.
	 */
	if (len == 0 && data == NULL) {
		buf[idx++] = 1;    /* param_count = 1 */
		buf[idx++] = 0x00; /* dummy byte */
	} else {
		buf[idx++] = (len == 256) ? 0 : (uint8_t)len;
		if (len > 0 && data) {
			size_t max_copy = sizeof(buf) - idx;
			size_t copy_len = (len > max_copy) ? max_copy : len;
			if (len > max_copy) {
				LOG_WRN("Truncating data copy: requested %zu bytes, only %zu fit "
					"in buffer",
					(size_t)len, max_copy);
			}
			memcpy(&buf[idx], data, copy_len);
			idx += copy_len;
		}
	}
	buf[idx++] = ack;

	int crc_len = idx; /* Length of data for CRC calculation */
	uint16_t crc = crc16(buf, crc_len);
	buf[idx++] = crc >> 8;
	buf[idx++] = crc & 0xFF;

	LOG_DBG("4WAY TX: cmd=0x%02X addr=0x%04X ack=0x%02X len=%d", cmd, addr, ack, idx);

	/* Send exactly like MSP does - direct poll_out, no delays between bytes */
	for (int i = 0; i < idx; i++) {
		uart_poll_out(usb_uart, buf[i]);
	}
}

static void fourway_send(uint8_t cmd, uint8_t ack, const uint8_t *data, uint16_t len)
{
	fourway_send_ex(cmd, 0, ack, data, len);
}

/* Process MSP command */
static void process_msp(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	LOG_DBG("MSP cmd=%d", cmd);

	switch (cmd) {
	case MSP_API_VERSION: {
		uint8_t resp[] = {0, 1, 46, 0}; /* MSP v1, API 1.46.0 */
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_FC_VARIANT: {
		uint8_t resp[] = {'B', 'T', 'F', 'L'};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_FC_VERSION: {
		uint8_t resp[] = {4, 5, 0};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_BOARD_INFO: {
		uint8_t resp[] = {'V', 'M', 'U', '1', 0, 0, 0, 0, 0};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_BUILD_INFO: {
		uint8_t resp[] = "Dec 14 2025 00:00:00";
		msp_send(cmd, resp, 19);
		break;
	}
	case MSP_MOTOR: {
		/* 8 motors x 2 bytes = 16 bytes (count = len/2 = 8) */
		uint8_t resp[16] = {0};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_BATTERY_STATE: {
		/* Battery: cells, cap, voltage, mAh, amps, state, voltage100 */
		uint8_t resp[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_MOTOR_CONFIG: {
		/* Motor config: minThrottle, maxThrottle, minCommand, motorCount, poles,
		 * dshot_telem, reserved */
		uint8_t resp[] = {
			0x2E,
			0x04, /* minThrottle = 1070 */
			0xD0,
			0x07, /* maxThrottle = 2000 */
			0xE8,
			0x03,             /* minCommand = 1000 */
			NUM_ESC_CHANNELS, /* motorCount */
			14,               /* motorPoles */
			1,                /* useDshotTelemetry */
			0                 /* reserved */
		};
		msp_send(cmd, resp, sizeof(resp));
		break;
	}
	case MSP_SET_4WAY_IF: {
		LOG_INF("Entering 4-way mode");
		uint8_t resp[] = {NUM_ESC_CHANNELS};
		msp_send(cmd, resp, sizeof(resp));
		/* Small delay to ensure response is sent before entering 4-way mode */
		k_msleep(10);
		/* Clear RX buffer to avoid stale data being interpreted as 4-way commands */
		rx_len = 0;
		fourway_mode = true;
		current_esc = 0;
		break;
	}
	default:
		LOG_WRN("Unknown MSP cmd: %d", cmd);
		msp_send(cmd, NULL, 0);
		break;
	}
}

/* Process 4-way command */
/* Request format: [0x2F] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [crc] */
static void process_fourway(const uint8_t *buf, int len)
{
	if (len < 5) {
		return;
	}

	/* buf[0] = 0x2F start marker, skip it */
	uint8_t cmd = buf[1];
	uint16_t addr = (buf[2] << 8) | buf[3];
	uint8_t param_count = buf[4];
	/* params start at buf[5] */

	LOG_DBG("4WAY cmd=0x%02X addr=0x%04X params=%d", cmd, addr, param_count);

	switch (cmd) {
	case FOURWAY_INTERFACE_TEST_ALIVE:
		fourway_send(cmd, ACK_OK, NULL, 0);
		break;

	case FOURWAY_PROTOCOL_GET_VERSION: {
		uint8_t ver = 108;
		fourway_send(cmd, ACK_OK, &ver, 1);
		break;
	}
	case FOURWAY_INTERFACE_GET_NAME: {
		const char *name = "m4wFCIntf";
		fourway_send(cmd, ACK_OK, (const uint8_t *)name, strlen(name));
		break;
	}
	case FOURWAY_INTERFACE_GET_VERSION: {
		uint8_t ver[] = {201 >> 8, 201 & 0xFF};
		fourway_send(cmd, ACK_OK, ver, 2);
		break;
	}
	case FOURWAY_INTERFACE_EXIT:
		LOG_INF("Exit 4-way mode");
		for (int i = 0; i < NUM_ESC_CHANNELS; i++) {
			esc_input(i);
		}
		fourway_send(cmd, ACK_OK, NULL, 0);
		fourway_mode = false;
		break;

	case FOURWAY_INTERFACE_SET_MODE: {
		uint8_t esc = (param_count >= 1) ? buf[5] : 0;
		if (esc >= NUM_ESC_CHANNELS) {
			fourway_send(cmd, ACK_INVALID_CHANNEL, NULL, 0);
		} else {
			current_esc = esc;
			LOG_DBG("Select ESC %d", current_esc);
			fourway_send(cmd, ACK_OK, NULL, 0);
		}
		break;
	}
	case FOURWAY_DEVICE_INIT_FLASH: {
		uint8_t esc = (param_count >= 1) ? buf[5] : 0;
		LOG_DBG("DeviceInitFlash: esc=%d", esc);

		/* Reset erase tracking for this ESC's new session */
		if (esc < NUM_ESC_CHANNELS) {
			eeprom_erased[esc] = false;
		}

		if (esc >= NUM_ESC_CHANNELS) {
			LOG_WRN("Invalid ESC channel %d", esc);
			fourway_send(cmd, ACK_INVALID_CHANNEL, NULL, 0);
			break;
		}
		current_esc = esc;

		/* BLHeli/AM32 bootloader query sequence (21 bytes):
		 * 12 zeros + 0x0D + "BLHeli" + checksum (0xF4 0x7D)
		 */
		static const uint8_t boot_init[] = {
			0,    0,   0,   0,   0,   0,   0, 0, 0, 0, 0, 0, /* 12 zeros */
			0x0D,                                            /* CR */
			'B',  'L', 'H', 'e', 'l', 'i',                   /* "BLHeli" */
			0xF4, 0x7D                                       /* checksum */
		};

		/* Send bootloader init sequence */
		esc_output(esc);
		for (size_t i = 0; i < sizeof(boot_init); i++) {
			sw_tx(esc, boot_init[i]);
		}
		esc_input(esc);

		LOG_DBG("Query sent, waiting for response...");

		/* Read 8-byte response:
		 * Bytes 0-2: "471" protocol ID
		 * Bytes 3-5: device info
		 * Bytes 6-7: additional info
		 *
		 * IMPORTANT: No logging inside this loop! Logging adds delay
		 * that disrupts the timing-sensitive bit-banged serial protocol.
		 */
		uint8_t resp[8] = {0};
		int got = 0;
		for (int i = 0; i < 12 && got < 8; i++) {
			uint8_t byte;
			if (sw_rx(esc, &byte, 50000) == 0) {
				resp[got++] = byte;
			}
		}

		/* Log response after all bytes received */
		LOG_DBG("ESC %d resp: [%02X %02X %02X %02X %02X %02X %02X %02X]", esc + 1, resp[0],
			resp[1], resp[2], resp[3], resp[4], resp[5], resp[6], resp[7]);

		if (got >= 8 && resp[0] == '4' && resp[1] == '7' && resp[2] == '1') {
			/* Valid BLHeli bootloader response - extract device info */
			uint16_t sig = (resp[4] << 8) | resp[5];
			uint8_t mode;
			if (sig >= 0x9307 && sig <= 0x930F) {
				mode = 0x01; /* SiLabs C2 interface */
			} else {
				mode = 0x04; /* ARM interface (default for AM32) */
			}
			uint8_t info[4];
			info[0] = resp[5]; /* signature LOW byte (0x06) */
			info[1] = resp[4]; /* signature HIGH byte (0x1F) */
			info[2] = resp[3]; /* input value (0x14) */
			info[3] = mode;    /* interface mode (4 = ARM) */
			LOG_INF("ESC %d: sig=0x%04X", esc + 1, sig);
			/* Response address field should be ESC index */
			fourway_send_ex(cmd, esc, ACK_OK, info, 4);
		} else if (got > 0) {
			/* Got some response but not valid - log it */
			LOG_WRN("ESC %d: invalid response (got %d bytes, first=%02X)", esc + 1, got,
				resp[0]);
			fourway_send(cmd, ACK_GENERAL_ERROR, NULL, 0);
		} else {
			LOG_WRN("ESC %d: no bootloader response", esc + 1);
			fourway_send(cmd, ACK_GENERAL_ERROR, NULL, 0);
		}
		break;
	}
	case FOURWAY_DEVICE_RESET:
		LOG_DBG("Reset ESC %d", current_esc);
		esc_output(current_esc);
		esc_low(current_esc);
		k_msleep(10);
		esc_high(current_esc);
		k_msleep(100);
		esc_input(current_esc);
		fourway_send(cmd, ACK_OK, NULL, 0);
		break;

	case FOURWAY_DEVICE_READ:
handle_device_read: {
	/* param_count is number of param bytes, buf[5] contains actual read length
	 * read_count: 0 = 256 bytes, otherwise the number of bytes to read
	 */
	uint8_t read_count = (param_count >= 1) ? buf[5] : 0;
	uint16_t rlen = (read_count == 0) ? 256 : read_count;
	uint8_t data[256];
	LOG_DBG("DeviceRead: %d bytes from 0x%04X", rlen, addr);

	int ret = bl_set_address(current_esc, addr);
	if (ret != 0) {
		LOG_WRN("Read: SET_ADDRESS failed");
		fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
		break;
	}

	ret = bl_read_data(current_esc, data, rlen);
	if (ret != 0) {
		LOG_WRN("Read: failed");
		fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
		break;
	}

	LOG_DBG("Read OK: %d bytes", rlen);
	fourway_send_ex(cmd, addr, ACK_OK, data, rlen);
	break;
}
	case FOURWAY_DEVICE_WRITE:
handle_device_write: {
	/* param_count is write length (0 means 256), data in params */
	uint16_t wlen = (param_count == 0) ? 256 : param_count;
	if (len < 5 + wlen) {
		LOG_WRN("DeviceWrite: buffer too short (%d < %d)", len, 5 + wlen);
		fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
		break;
	}
	LOG_DBG("DeviceWrite: %d bytes to 0x%04X (ESC %d)", wlen, addr, current_esc + 1);
	LOG_HEXDUMP_DBG(&buf[5], (wlen > 32) ? 32 : wlen, "Write data:");

	/* AM32 bootloader protocol:
	 * 1. SET_ADDRESS to set flash address
	 * 2. For EEPROM area (0x7C00), auto-erase page first
	 * 3. PROG_FLASH to write data
	 */
	int ret;

	/* Auto-erase EEPROM page before first write (0x7C00 area) - per ESC */
	if (addr >= 0x7C00 && addr < 0x8000 && !eeprom_erased[current_esc]) {
		LOG_DBG("Auto-erasing EEPROM page at 0x7C00 (ESC %d)", current_esc + 1);
		ret = bl_set_address(current_esc, 0x7C00);
		if (ret != 0) {
			LOG_WRN("DeviceWrite: SET_ADDRESS for erase failed");
			fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
			break;
		}
		ret = bl_erase_page(current_esc);
		if (ret != 0) {
			LOG_WRN("DeviceWrite: Auto-erase failed");
			fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
			break;
		}
		eeprom_erased[current_esc] = true; /* Don't erase again for this ESC */
		LOG_DBG("Auto-erase OK (ESC %d)", current_esc + 1);
		/* Small delay after erase before write */
		k_msleep(10);
	}

	ret = bl_set_address(current_esc, addr);
	if (ret != 0) {
		LOG_WRN("DeviceWrite: SET_ADDRESS failed");
		fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
		break;
	}

	ret = bl_write_data(current_esc, &buf[5], wlen);
	if (ret != 0) {
		LOG_WRN("DeviceWrite: PROG_FLASH failed");
		fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
		break;
	}

	LOG_DBG("Write OK");
	fourway_send_ex(cmd, addr, ACK_OK, NULL, 0);
	break;
}
	case FOURWAY_DEVICE_PAGE_ERASE: {
		LOG_DBG("PageErase: 0x%04X", addr);

		/* AM32 bootloader protocol:
		 * 1. SET_ADDRESS to set flash address
		 * 2. ERASE_FLASH to erase page at that address
		 */
		int ret = bl_set_address(current_esc, addr);
		if (ret != 0) {
			LOG_WRN("PageErase: SET_ADDRESS failed");
			fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
			break;
		}

		ret = bl_erase_page(current_esc);
		if (ret != 0) {
			LOG_WRN("PageErase: ERASE failed");
			fourway_send_ex(cmd, addr, ACK_GENERAL_ERROR, NULL, 0);
			break;
		}

		LOG_DBG("PageErase: OK");
		fourway_send_ex(cmd, addr, ACK_OK, NULL, 0);
		break;
	}
	case FOURWAY_DEVICE_READ_EEPROM:
		/* AM32 stores EEPROM in flash - use same read command */
		LOG_DBG("ReadEEPROM -> DeviceRead");
		goto handle_device_read;

	case FOURWAY_DEVICE_WRITE_EEPROM:
		/* AM32 stores EEPROM in flash - use same write command */
		LOG_DBG("WriteEEPROM: redirecting to DeviceWrite");
		goto handle_device_write;

	default:
		LOG_WRN("Unknown 4WAY cmd: 0x%02X", cmd);
		fourway_send(cmd, ACK_INVALID_CMD, NULL, 0);
		break;
	}
}

/* UART RX callback */
static volatile int rx_total_bytes = 0;
static void uart_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t byte;
			while (uart_fifo_read(dev, &byte, 1) > 0) {
				if (rx_len < RX_BUF_SIZE) {
					unsigned int key = irq_lock();
					rx_buf[rx_len++] = byte;
					irq_unlock(key);
					rx_total_bytes++;
				}
			}
		}
	}
}

/* Process received data */
static void process_rx(void)
{
	if (rx_len == 0) {
		return;
	}

	/* Disable IRQs while manipulating buffer to prevent race conditions */
	unsigned int key = irq_lock();
	int local_len = rx_len;

	/* Debug: show any received data in 4-way mode (use DBG to avoid timing issues) */
	if (fourway_mode && local_len > 0) {
		LOG_DBG("RX buf: %d bytes, first=%02X", local_len, rx_buf[0]);
	}

	/* MSP: $M< header */
	if (local_len >= 6 && rx_buf[0] == '$' && rx_buf[1] == 'M' && rx_buf[2] == '<') {
		uint8_t plen = rx_buf[3];
		uint8_t total = 6 + plen;
		if (local_len < total) {
			irq_unlock(key);
			return;
		}

		uint8_t cksum = 0;
		for (int i = 3; i < total - 1; i++) {
			cksum ^= rx_buf[i];
		}
		if (cksum == rx_buf[total - 1]) {
			process_msp(rx_buf[4], &rx_buf[5], plen);
		}
		memmove(rx_buf, &rx_buf[total], local_len - total);
		rx_len = local_len - total;
		irq_unlock(key);
		return;
	}

	/* 4-way commands - handle even when not in 4-way mode (for InterfaceExit recovery) */
	if (local_len > 0 && rx_buf[0] == FOURWAY_START_MARKER) {
		LOG_DBG("4WAY marker detected: rx_len=%d, 4way_mode=%d", local_len, fourway_mode);
	}
	if (local_len >= 7 && rx_buf[0] == FOURWAY_START_MARKER) {
		/* Calculate expected message length from param_count */
		uint8_t param_count = rx_buf[4];
		uint16_t actual_params = (param_count == 0) ? 256 : param_count;
		int expected_len = 5 + actual_params + 2; /* header(5) + params + crc(2) */

		if (local_len < expected_len) {
			/* Still waiting for more data */
			irq_unlock(key);
			return;
		}

		/* Log raw bytes for debugging */
		LOG_DBG("4WAY rx: %d/%d bytes [%02X %02X %02X %02X %02X %02X...]", local_len,
			expected_len, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
			local_len > 4 ? rx_buf[4] : 0, local_len > 5 ? rx_buf[5] : 0);

		/* Validate CRC for expected_len bytes */
		uint16_t rx_crc = (rx_buf[expected_len - 2] << 8) | rx_buf[expected_len - 1];
		uint16_t calc = crc16(rx_buf, expected_len - 2);

		if (rx_crc == calc) {
			LOG_DBG("4WAY CRC OK");
			/* Copy packet to local buffer before unlocking */
			uint8_t pkt[270];
			memcpy(pkt, rx_buf, expected_len);
			/* Remove processed bytes from buffer */
			if (local_len > expected_len) {
				memmove(rx_buf, &rx_buf[expected_len], local_len - expected_len);
				rx_len = local_len - expected_len;
			} else {
				rx_len = 0;
			}
			irq_unlock(key);
			/* Process packet with IRQs enabled (allows USB TX to work) */
			process_fourway(pkt, expected_len - 2);
		} else {
			LOG_WRN("4WAY CRC mismatch: expected 0x%04X, got 0x%04X", calc, rx_crc);
			/* Discard first byte and try again */
			memmove(rx_buf, &rx_buf[1], local_len - 1);
			rx_len = local_len - 1;
			irq_unlock(key);
		}
		return;
	}

	/* Not in 4-way mode and not MSP - discard */
	if (!fourway_mode && local_len > 64) {
		LOG_WRN("Discarding %d non-MSP bytes", local_len);
		rx_len = 0;
	}
	irq_unlock(key);
}

int main(void)
{
	LOG_INF("ESC Passthrough - MSP + 4-Way Interface");

	/* Apply pinctrl to set GPIO mux mode for ESC pins */
#if HAS_ESC_PINCTRL
	int pret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(esc_gpios)),
				       PINCTRL_STATE_DEFAULT);
	if (pret < 0) {
		LOG_ERR("Failed to apply ESC pinctrl: %d", pret);
	} else {
		LOG_INF("ESC GPIO pinctrl applied");
	}
#endif

	/* Init ESC GPIOs - all HIGH for bootloader entry */
	for (int i = 0; i < NUM_ESC_CHANNELS; i++) {
		if (!gpio_is_ready_dt(&esc_gpios[i])) {
			LOG_ERR("ESC GPIO %d not ready", i);
			return -1;
		}
		gpio_pin_configure_dt(&esc_gpios[i], GPIO_OUTPUT);
		gpio_pin_set_dt(&esc_gpios[i], 1); /* HIGH for bootloader entry */
		LOG_INF("ESC %d: GPIO1.%d = HIGH", i + 1, esc_gpios[i].pin);
	}

	LOG_INF("*** ALL ESC SIGNALS HIGH - PLUG IN BATTERY NOW ***");

	if (!device_is_ready(usb_uart)) {
		LOG_ERR("USB CDC not ready");
		return -1;
	}

	/* USB is auto-initialized at boot via CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT */
	LOG_INF("USB auto-initialized at boot");

	uart_irq_callback_set(usb_uart, uart_cb);
	uart_irq_rx_enable(usb_uart);

	LOG_INF("Ready - connect am32.ca configurator to /dev/ttyACM0");

	int loop_count = 0;
	while (1) {
		process_rx();
		k_msleep(1);

		loop_count++;

		/* Periodic status - disabled, enable for debugging */
		/* if (fourway_mode && (loop_count % 2000) == 0) {
			printk("[4WAY] status: rx_len=%d, total_rx=%d, loop=%d\n", rx_len,
		rx_total_bytes, loop_count);
		} */

		/* Log any pending data */
		if (fourway_mode && (loop_count % 500) == 0 && rx_len > 0) {
			LOG_DBG("4WAY pending: rx_len=%d, buf[0..7]: %02X %02X %02X %02X %02X %02X "
				"%02X %02X",
				rx_len, rx_buf[0], rx_len > 1 ? rx_buf[1] : 0,
				rx_len > 2 ? rx_buf[2] : 0, rx_len > 3 ? rx_buf[3] : 0,
				rx_len > 4 ? rx_buf[4] : 0, rx_len > 5 ? rx_buf[5] : 0,
				rx_len > 6 ? rx_buf[6] : 0, rx_len > 7 ? rx_buf[7] : 0);
		}
	}
}

/* Shell commands for testing and manual bootloader entry */

static int cmd_esc_low(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc low <channel 1-4>");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-4)");
		return -EINVAL;
	}
	esc_output(ch);
	esc_low(ch);
	/* Read back to verify */
	int val = gpio_pin_get_raw(esc_gpios[ch].port, esc_gpios[ch].pin);
	shell_print(sh, "ESC %d (GPIO1.%d) set LOW, readback=%d", ch + 1, esc_gpios[ch].pin, val);
	return 0;
}

static int cmd_esc_high(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc high <channel 1-4>");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-4)");
		return -EINVAL;
	}
	esc_output(ch);
	esc_high(ch);
	shell_print(sh, "ESC %d signal HIGH", ch + 1);
	return 0;
}

static int cmd_esc_input(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc input <channel 1-4>");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-4)");
		return -EINVAL;
	}
	esc_input(ch);
	shell_print(sh, "ESC %d set to input mode, reading: %d", ch + 1, esc_read(ch));
	return 0;
}

static int cmd_esc_test(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc test <channel 1-4>");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-4)");
		return -EINVAL;
	}
	shell_print(sh, "Testing ESC %d GPIO (GPIO1.%d)...", ch + 1, esc_gpios[ch].pin);
	esc_output(ch);
	shell_print(sh, "  LOW...");
	esc_low(ch);
	k_msleep(500);
	shell_print(sh, "  HIGH...");
	esc_high(ch);
	k_msleep(500);
	shell_print(sh, "  LOW...");
	esc_low(ch);
	k_msleep(500);
	esc_high(ch);
	esc_input(ch);
	shell_print(sh, "Test complete - check if motor beeped");
	return 0;
}

static int cmd_esc_boot(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc boot <channel 1-4 | all>");
		shell_print(sh, "Holds signal HIGH for bootloader entry (AM32 standard)");
		return -EINVAL;
	}

	bool boot_all = (strcmp(argv[1], "all") == 0);
	int ch = boot_all ? -1 : (atoi(argv[1]) - 1);

	if (!boot_all && (ch < 0 || ch >= NUM_ESC_CHANNELS)) {
		shell_print(sh, "Invalid channel (1-4 or 'all')");
		return -EINVAL;
	}

	if (boot_all) {
		shell_print(sh, "Holding ALL ESCs HIGH for bootloader entry...");
		for (int i = 0; i < NUM_ESC_CHANNELS; i++) {
			esc_output(i);
			esc_high(i);
		}
	} else {
		shell_print(sh, "Holding ESC %d HIGH for bootloader entry...", ch + 1);
		esc_output(ch);
		esc_high(ch);
	}

	shell_print(sh, ">>> PLUG IN BATTERY within 10 seconds <<<");

	/* Wait for battery to be plugged in */
	k_msleep(10000);

	shell_print(sh, "Querying bootloaders...");

	/* Query each ESC */
	int start_ch = boot_all ? 0 : ch;
	int end_ch = boot_all ? NUM_ESC_CHANNELS : (ch + 1);

	/* BLHeli/AM32 bootloader init sequence */
	static const uint8_t boot_init[] = {
		0,    0,   0,   0,   0,   0,   0, 0, 0, 0, 0, 0, /* 12 zeros */
		0x0D,                                            /* CR */
		'B',  'L', 'H', 'e', 'l', 'i',                   /* "BLHeli" */
		0xF4, 0x7D                                       /* checksum */
	};

	for (int esc = start_ch; esc < end_ch; esc++) {
		shell_print(sh, "ESC %d:", esc + 1);

		for (int query = 0; query < 3; query++) {
			/* Send bootloader init sequence */
			esc_output(esc);
			for (size_t i = 0; i < sizeof(boot_init); i++) {
				sw_tx(esc, boot_init[i]);
			}
			esc_input(esc);

			/* Read 8-byte response */
			uint8_t resp[8] = {0};
			int got = 0;
			for (int i = 0; i < 12 && got < 8; i++) {
				uint8_t byte;
				if (sw_rx(esc, &byte, 50000) == 0) {
					resp[got++] = byte;
				}
			}

			if (got >= 8 && resp[0] == '4' && resp[1] == '7' && resp[2] == '1') {
				shell_print(sh, "  Bootloader: %c%c%c %02X %02X %02X %02X %02X",
					    resp[0], resp[1], resp[2], resp[3], resp[4], resp[5],
					    resp[6], resp[7]);
				break;
			} else if (got > 0) {
				shell_print(sh, "  Got %d bytes (first=%02X), retrying...", got,
					    resp[0]);
			} else if (query == 2) {
				shell_print(sh, "  No bootloader response");
			}
		}

		k_msleep(100);
	}

	shell_print(sh, "Done. Now connect am32.ca configurator");
	return 0;
}

static int cmd_esc_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "ESC GPIO Status:");
	for (int i = 0; i < NUM_ESC_CHANNELS; i++) {
		int val = gpio_pin_get_raw(esc_gpios[i].port, esc_gpios[i].pin);
		shell_print(sh, "  ESC %d (GPIO1.%d): %d", i + 1, esc_gpios[i].pin, val);
	}
	return 0;
}

/* Test command to send a known-good 4-way response */
static int cmd_esc_test_tx(const struct shell *sh, size_t argc, char **argv)
{
	/* Build a DeviceInitFlash response with test data */
	uint8_t buf[20];
	int idx = 0;

	/* Response format: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack]
	 * [crc_hi] [crc_lo] */
	buf[idx++] = 0x2E; /* Response marker */
	buf[idx++] = 0x37; /* cmd = DeviceInitFlash */
	buf[idx++] = 0x00; /* addr_hi */
	buf[idx++] = 0x00; /* addr_lo */
	buf[idx++] = 0x04; /* param_count = 4 */
	buf[idx++] = 0xC1; /* Bootloader signature byte 0 */
	buf[idx++] = 0x00; /* Bootloader signature byte 1 */
	buf[idx++] = 0x00; /* Bootloader signature byte 2 */
	buf[idx++] = 0x00; /* Bootloader signature byte 3 */
	buf[idx++] = 0x00; /* ACK = ACK_OK */

	/* Calculate CRC on bytes 0 through idx-1 */
	uint16_t crc = crc16(buf, idx);
	buf[idx++] = crc >> 8;
	buf[idx++] = crc & 0xFF;

	shell_print(sh, "Sending test 4WAY response (%d bytes):", idx);
	shell_print(sh, "  Header: %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3],
		    buf[4]);
	shell_print(sh, "  Data:   %02X %02X %02X %02X", buf[5], buf[6], buf[7], buf[8]);
	shell_print(sh, "  ACK:    %02X", buf[9]);
	shell_print(sh, "  CRC:    %02X %02X (0x%04X)", buf[10], buf[11], crc);

	/* Send via USB CDC */
	for (int i = 0; i < idx; i++) {
		uart_poll_out(usb_uart, buf[i]);
	}

	shell_print(sh, "Sent! Check if configurator receives this.");
	return 0;
}

/* Hex dump test to see raw bytes received by USB */
static int cmd_esc_dump(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "4WAY mode: %s", fourway_mode ? "ACTIVE" : "inactive");
	shell_print(sh, "RX buffer (%d bytes):", rx_len);
	for (int i = 0; i < rx_len && i < 64; i += 16) {
		char hex[64];
		int pos = 0;
		for (int j = 0; j < 16 && i + j < rx_len; j++) {
			pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", rx_buf[i + j]);
		}
		shell_print(sh, "  %s", hex);
	}
	return 0;
}

/* Read EEPROM from ESC (initializes bootloader first) */
static int cmd_esc_eeprom(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc eeprom <channel 1-4>");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-4)");
		return -EINVAL;
	}

	shell_print(sh, "Initializing bootloader on ESC %d...", ch + 1);

	/* Send BLHeli/AM32 bootloader init sequence first */
	static const uint8_t boot_init[] = {
		0,    0,   0,   0,   0,   0,   0, 0, 0, 0, 0, 0, /* 12 zeros */
		0x0D,                                            /* CR */
		'B',  'L', 'H', 'e', 'l', 'i',                   /* "BLHeli" */
		0xF4, 0x7D                                       /* checksum */
	};

	esc_output(ch);
	for (size_t i = 0; i < sizeof(boot_init); i++) {
		sw_tx(ch, boot_init[i]);
	}
	esc_input(ch);

	/* Read 8-byte response */
	uint8_t resp[8] = {0};
	int got = 0;
	for (int i = 0; i < 12 && got < 8; i++) {
		uint8_t byte;
		if (sw_rx(ch, &byte, 50000) == 0) {
			resp[got++] = byte;
		}
	}

	if (got < 8 || resp[0] != '4' || resp[1] != '7' || resp[2] != '1') {
		shell_print(sh, "Bootloader init failed (got %d bytes, first=%02X)", got, resp[0]);
		shell_print(sh, "Make sure ESC is powered with signal HIGH for bootloader entry");
		return -EIO;
	}

	uint16_t sig = (resp[4] << 8) | resp[5];
	shell_print(sh, "Bootloader OK: sig=0x%04X", sig);
	shell_print(sh, "");
	shell_print(sh, "Reading EEPROM from ESC %d...", ch + 1);

	/* Read firmware info area (32 bytes before EEPROM) */
	uint8_t fw_info[32];
	int ret = bl_set_address(ch, 0x7BE0);
	if (ret != 0) {
		shell_print(sh, "SET_ADDRESS 0x7BE0 failed");
		return ret;
	}
	ret = bl_read_data(ch, fw_info, 32);
	if (ret != 0) {
		shell_print(sh, "READ 0x7BE0 failed");
		return ret;
	}
	shell_print(sh, "FW Info (0x7BE0):");
	shell_hexdump(sh, fw_info, 32);

	/* Read EEPROM (184 bytes = 0xB8 at 0x7C00) - full AM32 settings layout */
	uint8_t eeprom[184];
	ret = bl_set_address(ch, 0x7C00);
	if (ret != 0) {
		shell_print(sh, "SET_ADDRESS 0x7C00 failed");
		return ret;
	}
	ret = bl_read_data(ch, eeprom, 184);
	if (ret != 0) {
		shell_print(sh, "READ 0x7C00 failed (184 bytes)");
		return ret;
	}
	shell_print(sh, "EEPROM (0x7C00, 184 bytes):");
	shell_hexdump(sh, eeprom, 184);

	/* Parse key fields */
	shell_print(sh, "");
	shell_print(sh, "Parsed EEPROM:");
	shell_print(sh, "  Boot indicator: 0x%02X (%s)", eeprom[0],
		    eeprom[0] == 0x01 ? "VALID" : "INVALID");
	shell_print(sh, "  EEPROM version: %d", eeprom[1]);
	shell_print(sh, "  Bootloader ver: %d", eeprom[2]);
	shell_print(sh, "  FW version: %d.%d", eeprom[3], eeprom[4]);

	/* Display ESC name - filter out 0xFF (uninitialized) bytes */
	char name_buf[13] = {0};
	bool name_valid = false;
	for (int i = 0; i < 12; i++) {
		uint8_t c = eeprom[5 + i];
		if (c != 0xFF && c != 0x00 && c >= 0x20 && c < 0x7F) {
			name_buf[i] = c;
			name_valid = true;
		} else {
			name_buf[i] = '\0';
			break;
		}
	}
	shell_print(sh, "  ESC name: %s", name_valid ? name_buf : "(not set)");

	shell_print(sh, "  Reversed: %d", eeprom[17]);
	shell_print(sh, "  Bidirectional: %d", eeprom[18]);

	return 0;
}

/* Query bootloader only (no EEPROM read) - quick test */
static int cmd_esc_query(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc query <channel 1-%d>", NUM_ESC_CHANNELS);
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel (1-%d)", NUM_ESC_CHANNELS);
		return -EINVAL;
	}

	shell_print(sh, "Querying bootloader on ESC %d...", ch + 1);

	static const uint8_t boot_init[] = {0, 0,    0,   0,   0,   0,   0,   0,   0,    0,   0,
					    0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};

	esc_output(ch);
	for (size_t i = 0; i < sizeof(boot_init); i++) {
		sw_tx(ch, boot_init[i]);
	}
	esc_input(ch);

	uint8_t resp[8] = {0};
	int got = 0;
	for (int i = 0; i < 12 && got < 8; i++) {
		uint8_t byte;
		if (sw_rx(ch, &byte, 50000) == 0) {
			resp[got++] = byte;
			shell_print(sh, "  [%d] 0x%02X '%c'", got, byte,
				    (byte >= 0x20 && byte < 0x7F) ? byte : '.');
		}
	}

	if (got >= 8 && resp[0] == '4' && resp[1] == '7' && resp[2] == '1') {
		uint16_t sig = (resp[4] << 8) | resp[5];
		shell_print(sh, "SUCCESS: sig=0x%04X input=0x%02X", sig, resp[3]);
		return 0;
	} else if (got > 0) {
		shell_print(sh, "FAIL: got %d bytes, not valid 471 response", got);
		shell_print(sh, "  Response: %02X %02X %02X...", resp[0], resp[1], resp[2]);
		return -EIO;
	} else {
		shell_print(sh, "FAIL: no response (ESC powered? signal was HIGH at boot?)");
		return -ETIMEDOUT;
	}
}

/* Test write to EEPROM - writes and verifies a test pattern */
static int cmd_esc_write_test(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: esc write_test <channel 1-%d>", NUM_ESC_CHANNELS);
		shell_print(sh, "WARNING: This modifies EEPROM! Use with caution.");
		return -EINVAL;
	}
	int ch = atoi(argv[1]) - 1;
	if (ch < 0 || ch >= NUM_ESC_CHANNELS) {
		shell_print(sh, "Invalid channel");
		return -EINVAL;
	}

	/* First query bootloader */
	shell_print(sh, "Step 1: Query bootloader...");
	static const uint8_t boot_init[] = {0, 0,    0,   0,   0,   0,   0,   0,   0,    0,   0,
					    0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
	esc_output(ch);
	for (size_t i = 0; i < sizeof(boot_init); i++) {
		sw_tx(ch, boot_init[i]);
	}
	esc_input(ch);

	uint8_t resp[8] = {0};
	int got = 0;
	for (int i = 0; i < 12 && got < 8; i++) {
		uint8_t byte;
		if (sw_rx(ch, &byte, 50000) == 0) {
			resp[got++] = byte;
		}
	}
	if (got < 8 || resp[0] != '4') {
		shell_print(sh, "  FAIL: Bootloader not responding");
		return -EIO;
	}
	shell_print(sh, "  OK: Bootloader ready");

	/* Read current value at test address */
	shell_print(sh, "Step 2: Read current EEPROM value...");
	uint16_t test_addr = 0x7C00; /* First byte of EEPROM */
	int ret = bl_set_address(ch, test_addr);
	if (ret != 0) {
		shell_print(sh, "  FAIL: SET_ADDRESS returned %d", ret);
		return ret;
	}
	uint8_t original[1];
	ret = bl_read_data(ch, original, 1);
	if (ret != 0) {
		shell_print(sh, "  FAIL: READ returned %d", ret);
		return ret;
	}
	shell_print(sh, "  OK: Current value = 0x%02X", original[0]);

	/* Write test value */
	uint8_t test_val = (original[0] == 0x42) ? 0x43 : 0x42;
	shell_print(sh, "Step 3: Write test value 0x%02X...", test_val);
	ret = bl_set_address(ch, test_addr);
	if (ret != 0) {
		shell_print(sh, "  FAIL: SET_ADDRESS for write returned %d", ret);
		return ret;
	}
	uint8_t write_data[1] = {test_val};
	ret = bl_write_data(ch, write_data, 1);
	if (ret != 0) {
		shell_print(sh, "  FAIL: WRITE returned %d", ret);
		return ret;
	}
	shell_print(sh, "  OK: Write completed");

	/* Verify by reading back */
	shell_print(sh, "Step 4: Verify write...");
	ret = bl_set_address(ch, test_addr);
	if (ret != 0) {
		shell_print(sh, "  FAIL: SET_ADDRESS for verify returned %d", ret);
		return ret;
	}
	uint8_t verify[1];
	ret = bl_read_data(ch, verify, 1);
	if (ret != 0) {
		shell_print(sh, "  FAIL: READ for verify returned %d", ret);
		return ret;
	}

	if (verify[0] == test_val) {
		shell_print(sh, "  OK: Verified 0x%02X", verify[0]);
		shell_print(sh, "");
		shell_print(sh, "WRITE TEST PASSED!");
	} else {
		shell_print(sh, "  FAIL: Expected 0x%02X, got 0x%02X", test_val, verify[0]);
		return -EIO;
	}

	/* Restore original value */
	shell_print(sh, "Step 5: Restore original value 0x%02X...", original[0]);
	ret = bl_set_address(ch, test_addr);
	if (ret == 0) {
		write_data[0] = original[0];
		ret = bl_write_data(ch, write_data, 1);
	}
	if (ret == 0) {
		shell_print(sh, "  OK: Restored");
	} else {
		shell_print(sh, "  WARN: Restore failed, EEPROM modified");
	}

	return 0;
}

/* Show 4-way mode state for debugging */
static int cmd_esc_4way_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "4-Way Interface Status:");
	shell_print(sh, "  Mode: %s", fourway_mode ? "ACTIVE" : "inactive");
	shell_print(sh, "  Current ESC: %d", current_esc + 1);
	shell_print(sh, "  USB device: %s", device_is_ready(usb_uart) ? "ready" : "NOT READY");
	shell_print(sh, "  ESC channels: %d", NUM_ESC_CHANNELS);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_esc, SHELL_CMD(low, NULL, "Set ESC signal LOW", cmd_esc_low),
	SHELL_CMD(high, NULL, "Set ESC signal HIGH", cmd_esc_high),
	SHELL_CMD(input, NULL, "Set ESC to input mode", cmd_esc_input),
	SHELL_CMD(test, NULL, "Toggle ESC signal (motor should beep)", cmd_esc_test),
	SHELL_CMD(boot, NULL, "Manual bootloader entry (10s window)", cmd_esc_boot),
	SHELL_CMD(status, NULL, "Show GPIO status", cmd_esc_status),
	SHELL_CMD(test_tx, NULL, "Send test 4WAY response to USB", cmd_esc_test_tx),
	SHELL_CMD(dump, NULL, "Dump RX buffer contents", cmd_esc_dump),
	SHELL_CMD(eeprom, NULL, "Read EEPROM from ESC in bootloader mode", cmd_esc_eeprom),
	SHELL_CMD(query, NULL, "Query ESC bootloader (quick test)", cmd_esc_query),
	SHELL_CMD(write_test, NULL, "Test write to EEPROM (modifies data!)", cmd_esc_write_test),
	SHELL_CMD(4way, NULL, "Show 4-way interface status", cmd_esc_4way_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(esc, &sub_esc, "ESC GPIO commands", NULL);
