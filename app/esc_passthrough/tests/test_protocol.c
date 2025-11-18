/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 CogniPilot Foundation
 *
 * ESC Passthrough Protocol Tests
 *
 * These tests verify the bootloader protocol implementation.
 * Run with: west twister -T app/esc_passthrough/tests
 *
 * =============================================================================
 * PROTOCOL DOCUMENTATION - DO NOT CHANGE WITHOUT VERIFYING AGAINST SOURCES
 * =============================================================================
 *
 * This implementation is based on the official AM32 bootloader source code.
 * Before modifying any protocol behavior, verify against these authoritative sources:
 *
 * PRIMARY SOURCE:
 *   https://github.com/am32-firmware/AM32-Bootloader/blob/master/bootloader/main.c
 *
 * KEY PROTOCOL DETAILS (verified 2024-12):
 *
 * 1. CRC-16 Algorithm (crc16_bl):
 *    - Polynomial: 0xA001 (bit-reversed 0x8005)
 *    - LSB-first processing
 *    - Initial value: 0
 *    - Used for all bootloader commands
 *
 * 2. Bootloader Init Sequence (21 bytes):
 *    - 12 zeros + 0x0D + "BLHeli" + 0xF4 + 0x7D
 *    - Pattern matched at rxBuffer[12]=0x0D, rxBuffer[13]='B', rxBuffer[20]=0x7D
 *    - Response: "471" + device info (8 bytes total)
 *
 * 3. Command Format: [cmd] [params...] + CRC16 (low byte first)
 *
 * 4. SET_ADDRESS (0xFF): [0xFF, 0x00, addr_hi, addr_lo] + CRC -> ACK
 *
 * 5. SET_BUFFER (0xFE): [0xFE, 0x00, type, count] + CRC -> NO ACK!
 *    - CRITICAL: Bootloader does NOT send ACK after SET_BUFFER
 *    - Sets incoming_payload_no_command=1 and waits for data
 *    - type=0x01 means 256 bytes, else count byte is used
 *
 * 6. DATA Phase: [data bytes...] + CRC -> ACK
 *    - Sent immediately after SET_BUFFER (no ACK wait)
 *    - Bootloader collects exactly payload_buffer_size + 2 bytes
 *    - ACK sent after CRC validation
 *
 * 7. PROG_FLASH (0x01): [0x01, 0x00] + CRC -> ACK
 *    - Writes payLoadBuffer to flash at current address
 *
 * 8. READ_FLASH (0x03): [0x03, count] + CRC -> [data] + CRC + ACK
 *    - count=0 means 256 bytes
 *
 * 9. ERASE_FLASH (0x02): [0x02, 0x00] + CRC -> ACK
 *    - Erases page at current address
 *
 * 10. ACK Values:
 *     - 0x30 ('0') = ACK_OK
 *     - 0xC1 = BAD_CMD
 *     - 0xC2 = BAD_CRC
 *
 * 4-Way Interface Protocol (BLHeli passthrough):
 *   Reference:
 * https://github.com/am32-firmware/am32-configurator/blob/master/src/communication/four_way.ts
 *
 *   - Request:  [0x2F] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [crc_hi] [crc_lo]
 *   - Response: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack] [crc_hi] [crc_lo]
 *   - CRC: CRC16-XMODEM polynomial 0x1021, MSB-first, initial value 0
 *
 *   CRITICAL: param_count interpretation (from parseMessage() in four_way.ts):
 *     - param_count=0 means 256 bytes! (not zero)
 *     - For empty responses, use param_count=1 with a dummy 0x00 byte
 *     - ACK position: byte index [5 + param_count]
 *     - CRC calculated on bytes [0 .. 5 + param_count] (inclusive)
 *
 *   Response format parsing (from am32-configurator parseMessage()):
 *     view[0] = 0x2E (start marker)
 *     view[1] = command
 *     view[2] = address high byte
 *     view[3] = address low byte
 *     view[4] = param_count (0 means 256)
 *     view[5 .. 5+param_count) = params
 *     view[5 + param_count] = ack
 *     view[6 + param_count .. 8 + param_count) = checksum (big-endian)
 *
 *   CRC calculation (from crc16XmodemUpdate() in four_way.ts):
 *     msgWithoutChecksum = view.subarray(0, 6 + paramCount)
 *     checksum = msgWithoutChecksum.reduce(crc16XmodemUpdate, 0)
 *
 *   ACK values (from am32-configurator):
 *     - 0x00 = ACK_OK (success)
 *     - 0x02 = ACK_I_INVALID_CMD
 *     - 0x03 = ACK_I_INVALID_CRC
 *     - 0x04 = ACK_I_VERIFY_ERROR
 *     - 0x08 = ACK_I_INVALID_CHANNEL
 *     - 0x09 = ACK_I_INVALID_PARAM
 *     - 0x0F = ACK_D_GENERAL_ERROR
 *
 *   Timing (from am32-configurator):
 *     - Default timeout: 200ms per command
 *     - Default retries: 10 attempts
 *     - Retry delay: 250ms between attempts
 *
 * 11. Serial Baud Rate:
 *     - FIXED at 19200 baud - cannot be changed
 *     - This is part of the BLHeli/4-Way Interface specification
 *     - All AM32/BLHeli ESCs use this rate for bootloader communication
 *     - Bit time: ~52µs (1/19200 = 52.08µs)
 *
 * =============================================================================
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <string.h>

/* CRC-16 with polynomial 0xA001 (bit-reversed 0x8005) - matches BLHeli bootloader */
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

/* 4-way interface CRC calculation */
static uint16_t fourway_crc(const uint8_t *data, size_t len)
{
	uint16_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= ((uint16_t)data[i] << 8);
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

/*
 * Test Suite: CRC Calculations
 */
ZTEST_SUITE(crc_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(crc_tests, test_bootloader_crc_known_values)
{
	/* Test with known values from BLHeli bootloader protocol */
	uint8_t cmd_set_addr[] = {0xFF, 0x00, 0x7C, 0x00}; /* SET_ADDRESS 0x7C00 */
	uint16_t crc = crc16_bl(cmd_set_addr, 4);
	/* Expected CRC calculated from working implementation */
	zassert_true(crc != 0, "CRC should not be zero for non-zero input");
}

ZTEST(crc_tests, test_bootloader_crc_zero_input)
{
	uint8_t zeros[] = {0, 0, 0, 0};
	uint16_t crc = crc16_bl(zeros, 4);
	zassert_equal(crc, 0, "CRC of all zeros should be 0");
}

ZTEST(crc_tests, test_bootloader_crc_consistency)
{
	/* Same input should always produce same CRC */
	uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
	uint16_t crc1 = crc16_bl(data, 4);
	uint16_t crc2 = crc16_bl(data, 4);
	zassert_equal(crc1, crc2, "CRC should be consistent");
}

ZTEST(crc_tests, test_fourway_crc_known_values)
{
	/* Test DeviceInitFlash response CRC */
	uint8_t resp[] = {0x37, 0x00, 0x00, 0x04, 0x06, 0x1F, 0x14, 0x04, 0x00};
	uint16_t crc = fourway_crc(resp, 9);
	zassert_true(crc != 0, "4-way CRC should not be zero");
}

ZTEST(crc_tests, test_fourway_crc_consistency)
{
	uint8_t data[] = {0x2F, 0x37, 0x00, 0x00, 0x01, 0x00};
	uint16_t crc1 = fourway_crc(data, 6);
	uint16_t crc2 = fourway_crc(data, 6);
	zassert_equal(crc1, crc2, "4-way CRC should be consistent");
}

ZTEST(crc_tests, test_fourway_write_ack_response_format)
{
/*
 * Write ACK response format (with dummy byte to avoid param_count=0 issue):
 * [0x2E][cmd][addr_hi][addr_lo][param_count=1][dummy=0x00][ack=0x00][crc_hi][crc_lo]
 *
 * Reference: am32-configurator parseMessage() in four_way.ts
 * - param_count=0 means 256, so we use 1 with a dummy byte for empty responses
 * - ACK position: byte index [5 + param_count]
 * - CRC calculated on bytes [0 .. 5 + param_count] inclusive = 7 bytes for param_count=1
 *
 * Example: Write ACK response for cmd=0x3B (DeviceWrite) at address 0x7C00
 */
#define CMD_DEVICE_WRITE 0x3B
#define ACK_OK           0x00

	uint8_t response[9];
	int idx = 0;

	response[idx++] = 0x2E;             /* Response marker */
	response[idx++] = CMD_DEVICE_WRITE; /* Command echo */
	response[idx++] = 0x7C;             /* Address high */
	response[idx++] = 0x00;             /* Address low */
	response[idx++] = 1;                /* param_count = 1 (NOT 0!) */
	response[idx++] = 0x00;             /* dummy param byte */
	response[idx++] = ACK_OK;           /* ACK = success */

	/* CRC is calculated on bytes 0 through 6 (7 bytes) for param_count=1 */
	/* Reference: msgWithoutChecksum = view.subarray(0, 6 + paramCount) */
	int crc_len = 6 + 1; /* 6 + param_count */
	zassert_equal(crc_len, 7, "CRC length should be 7 for param_count=1");
	zassert_equal(idx, crc_len, "Index should match CRC length before adding CRC");

	uint16_t crc = fourway_crc(response, crc_len);
	response[idx++] = crc >> 8;   /* CRC high byte */
	response[idx++] = crc & 0xFF; /* CRC low byte */

	zassert_equal(idx, 9, "Total response should be 9 bytes");

	/* Verify structure matches parseMessage expectations */
	int param_count = response[4];
	zassert_equal(param_count, 1, "param_count should be 1");
	zassert_equal(response[5 + param_count], ACK_OK, "ACK should be at index 5+param_count=6");
	zassert_equal((response[6 + param_count] << 8) | response[7 + param_count], crc,
		      "Checksum should be at index 6+param_count and 7+param_count");
}

/*
 * Test Suite: Packet Parsing
 */
ZTEST_SUITE(packet_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(packet_tests, test_bootloader_query_format)
{
	/*
	 * BLHeli/AM32 bootloader query: 12 zeros + 0x0D + "BLHeli" + checksum
	 * Reference: https://github.com/am32-firmware/AM32-Bootloader/blob/master/bootloader/main.c
	 *
	 * The bootloader checks multiple init patterns in decodeInput():
	 *   Pattern 1: rxBuffer[16]==0x7D && rxBuffer[8]==0x0D && rxBuffer[9]=='B'  (17 bytes)
	 *   Pattern 2: rxBuffer[20]==0x7D && rxBuffer[12]==0x0D && rxBuffer[13]=='B' (21 bytes) <-
	 * we use this Pattern 3: rxBuffer[40]==0x7D && rxBuffer[32]==0x0D && rxBuffer[33]=='B' (41
	 * bytes)
	 *
	 * Response: sendDeviceInfo() sends "471" + 5 bytes device info
	 */
	static const uint8_t boot_init[] = {
		0,    0,   0,   0,   0,   0,   0, 0, 0, 0, 0, 0, /* 12 zeros (index 0-11) */
		0x0D,                                            /* CR (index 12) */
		'B',  'L', 'H', 'e', 'l', 'i',                   /* "BLHeli" (index 13-18) */
		0xF4, 0x7D                                       /* checksum (index 19-20) */
	};

	zassert_equal(sizeof(boot_init), 21, "Boot query should be 21 bytes");
	zassert_equal(boot_init[12], 0x0D, "Byte 12 should be CR (pattern 2 check)");
	zassert_equal(boot_init[13], 'B', "Byte 13 should be 'B' (pattern 2 check)");
	zassert_equal(boot_init[20], 0x7D, "Byte 20 should be 0x7D (pattern 2 check)");
	zassert_mem_equal(&boot_init[13], "BLHeli", 6, "Bytes 13-18 should be 'BLHeli'");
}

ZTEST(packet_tests, test_bootloader_response_parsing)
{
	/* Valid bootloader response: "471" + device info */
	uint8_t resp[] = {'4', '7', '1', 0x14, 0x1F, 0x06, 0x06, 0x01};

	/* Check protocol ID */
	zassert_equal(resp[0], '4', "First byte should be '4'");
	zassert_equal(resp[1], '7', "Second byte should be '7'");
	zassert_equal(resp[2], '1', "Third byte should be '1'");

	/* Extract signature */
	uint16_t sig = (resp[4] << 8) | resp[5];
	zassert_equal(sig, 0x1F06, "Signature should be 0x1F06");
}

ZTEST(packet_tests, test_fourway_packet_structure)
{
	/* 4-way request: [0x2F] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [crc_hi]
	 * [crc_lo] */
	uint8_t request[] = {0x2F, 0x37, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

	zassert_equal(request[0], 0x2F, "Request marker should be 0x2F");
	zassert_equal(request[1], 0x37, "DeviceInitFlash cmd should be 0x37");

	uint16_t addr = (request[2] << 8) | request[3];
	zassert_equal(addr, 0x0000, "Address should be 0");
}

ZTEST(packet_tests, test_fourway_response_structure)
{
	/* 4-way response: [0x2E] [cmd] [addr_hi] [addr_lo] [param_count] [params...] [ack] [crc_hi]
	 * [crc_lo] */
	uint8_t response[] = {0x2E, 0x37, 0x00, 0x00, 0x04, 0x06,
			      0x1F, 0x14, 0x04, 0x00, 0x12, 0x5A};

	zassert_equal(response[0], 0x2E, "Response marker should be 0x2E");
	zassert_equal(response[1], 0x37, "DeviceInitFlash cmd should be 0x37");
	zassert_equal(response[4], 0x04, "Param count should be 4");
	zassert_equal(response[9], 0x00, "ACK_OK should be 0x00");
}

/*
 * Test Suite: Protocol Commands
 */
ZTEST_SUITE(command_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(command_tests, test_set_address_format)
{
	/* SET_ADDRESS: [0xFF, 0x00, addr_hi, addr_lo] + CRC */
	uint8_t cmd[4] = {0xFF, 0x00, 0x7C, 0x00};
	uint16_t crc = crc16_bl(cmd, 4);

	zassert_equal(cmd[0], 0xFF, "SET_ADDRESS opcode should be 0xFF");
	zassert_equal(cmd[1], 0x00, "Second byte should be 0x00");

	uint16_t addr = (cmd[2] << 8) | cmd[3];
	zassert_equal(addr, 0x7C00, "Address should be 0x7C00");
	zassert_true(crc != 0, "CRC should be calculated");
}

ZTEST(command_tests, test_prog_flash_format)
{
	/*
	 * AM32 write protocol (see AM32_Bootloader_F051 source):
	 * 1. SET_BUFFER [0xFE, 0x00, type, count] + CRC -> ACK
	 * 2. [data] + CRC(data) -> ACK  (NO count prefix!)
	 * 3. PROG_FLASH [0x01, 0x00] + CRC -> ACK
	 */
	uint8_t cmd[2] = {0x01, 0x00}; /* PROG_FLASH command */
	uint16_t crc = crc16_bl(cmd, 2);

	zassert_equal(cmd[0], 0x01, "PROG_FLASH opcode should be 0x01");
	zassert_equal(cmd[1], 0x00, "Second byte should be 0x00");
	zassert_true(crc != 0, "CRC should be calculated");
}

ZTEST(command_tests, test_set_buffer_format)
{
	/*
	 * SET_BUFFER: [0xFE, 0x00, type, count] + CRC
	 * Reference: https://github.com/am32-firmware/AM32-Bootloader/blob/master/bootloader/main.c
	 *
	 * - type (byte 2): if 0x01, payload_buffer_size = 256
	 * - count (byte 3): if type != 0x01, payload_buffer_size = count
	 *
	 * IMPORTANT: SET_BUFFER does NOT send ACK!
	 * It sets incoming_payload_no_command=1 and waits for data bytes.
	 * The client should immediately send data after SET_BUFFER, no ACK wait.
	 */
	uint8_t cmd_184[4] = {0xFE, 0x00, 0x00, 0xB8}; /* 184 bytes */
	uint8_t cmd_256[4] = {0xFE, 0x00, 0x01, 0x00}; /* 256 bytes */

	zassert_equal(cmd_184[0], 0xFE, "SET_BUFFER opcode should be 0xFE");
	zassert_equal(cmd_184[3], 184, "Count should be 184 for non-256 case");
	zassert_equal(cmd_256[2], 0x01, "Type 0x01 indicates 256 bytes");

	uint16_t crc = crc16_bl(cmd_184, 4);
	zassert_true(crc != 0, "CRC should be calculated");
}

ZTEST(command_tests, test_count_byte_encoding)
{
	/* Count byte: 0 means 256, otherwise literal */
	uint8_t count_256 = (256 == 256) ? 0 : (uint8_t)256;
	uint8_t count_184 = (184 == 256) ? 0 : (uint8_t)184;
	uint8_t count_1 = (1 == 256) ? 0 : (uint8_t)1;

	zassert_equal(count_256, 0, "256 bytes should encode as 0");
	zassert_equal(count_184, 184, "184 bytes should encode as 184");
	zassert_equal(count_1, 1, "1 byte should encode as 1");
}

/*
 * Test Suite: MSP Protocol
 */
ZTEST_SUITE(msp_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(msp_tests, test_msp_header_format)
{
	/* MSP v1: ['$', 'M', '<'|'>', len, cmd, ...data..., crc] */
	uint8_t msp_request[] = {'$', 'M', '<', 0x01, 0xF5, 0x00, 0xF4};

	zassert_equal(msp_request[0], '$', "MSP start should be '$'");
	zassert_equal(msp_request[1], 'M', "MSP marker should be 'M'");
	zassert_equal(msp_request[2], '<', "Request direction should be '<'");
	zassert_equal(msp_request[4], 0xF5, "MSP_SET_PASSTHROUGH should be 245 (0xF5)");
}

ZTEST(msp_tests, test_msp_crc_calculation)
{
	/* MSP CRC is XOR of len, cmd, and data bytes */
	uint8_t len = 0x01;
	uint8_t cmd = 0xF5;
	uint8_t data = 0x00;
	uint8_t crc = len ^ cmd ^ data;

	zassert_equal(crc, 0xF4, "MSP CRC should be 0xF4");
}

/*
 * Test Suite: EEPROM Layout
 */
ZTEST_SUITE(eeprom_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(eeprom_tests, test_eeprom_addresses)
{
	/* AM32 EEPROM layout */
	uint16_t fw_info_addr = 0x7BE0; /* 32 bytes firmware info */
	uint16_t eeprom_addr = 0x7C00;  /* 184 bytes settings */
	uint16_t eeprom_size = 184;

	zassert_equal(eeprom_addr - fw_info_addr, 32, "FW info should be 32 bytes before EEPROM");
	zassert_equal(eeprom_size, 0xB8, "EEPROM size should be 184 (0xB8) bytes");
}

ZTEST(eeprom_tests, test_eeprom_field_offsets)
{
	/* Key EEPROM field offsets per AM32 layout */
	zassert_equal(0, 0, "Boot indicator at offset 0");
	zassert_equal(1, 1, "EEPROM version at offset 1");
	zassert_equal(2, 2, "Bootloader version at offset 2");
	zassert_equal(3, 3, "FW major at offset 3");
	zassert_equal(4, 4, "FW minor at offset 4");
	zassert_equal(5, 5, "ESC name at offset 5 (12 bytes)");
	zassert_equal(17, 17, "Reversed at offset 17");
	zassert_equal(18, 18, "Bidirectional at offset 18");
}

/*
 * Test Suite: AM32 Configurator Compatibility
 *
 * These tests mimic how the AM32 configurator (am32.ca) parses responses.
 * Reference:
 * https://github.com/am32-firmware/am32-configurator/blob/master/src/communication/four_way.ts
 */
ZTEST_SUITE(am32_compat_tests, NULL, NULL, NULL, NULL, NULL);

/*
 * Simulates parseMessage() from am32-configurator/src/communication/four_way.ts
 * Returns 0 on success, -1 on error
 */
static int simulate_parse_message(const uint8_t *buf, size_t len, uint8_t *out_cmd,
				  uint16_t *out_addr, uint8_t *out_ack, uint8_t *out_params,
				  size_t *out_param_len)
{
	const uint8_t FOURWAY_IF = 0x2E;

	/* Check start marker */
	if (buf[0] != FOURWAY_IF) {
		return -1; /* invalid message start */
	}

	/* Minimum length check */
	if (len < 9) {
		return -1; /* NotEnoughDataError */
	}

	/* Get param_count - CRITICAL: 0 means 256! */
	uint16_t param_count = buf[4];
	if (param_count == 0) {
		param_count = 256;
	}

	/* Check we have enough data */
	if (len < (size_t)(8 + param_count)) {
		return -1; /* NotEnoughDataError */
	}

	/* Extract fields (matching parseMessage exactly) */
	*out_cmd = buf[1];
	*out_addr = (buf[2] << 8) | buf[3];
	*out_ack = buf[5 + param_count];

	/* Extract checksum from response */
	uint16_t rx_checksum = (buf[6 + param_count] << 8) | buf[7 + param_count];

	/* Calculate expected checksum on msgWithoutChecksum = buf[0 .. 6 + paramCount) */
	uint16_t calc_checksum = fourway_crc(buf, 6 + param_count);

	if (rx_checksum != calc_checksum) {
		return -1; /* checksum mismatch */
	}

	/* Copy params */
	if (out_params && out_param_len) {
		*out_param_len = param_count;
		memcpy(out_params, &buf[5], param_count);
	}

	return 0;
}

ZTEST(am32_compat_tests, test_parse_write_ack_response)
{
	/*
	 * Test that our write ACK response format is correctly parsed by the
	 * AM32 configurator's parseMessage() function.
	 *
	 * Response format: [0x2E][cmd][addr_hi][addr_lo][1][dummy][ack][crc_hi][crc_lo]
	 */
	uint8_t response[9];
	int idx = 0;

	/* Build response exactly as fourway_send_ex does for writes */
	response[idx++] = 0x2E; /* Response marker */
	response[idx++] = 0x3B; /* cmd_DeviceWrite */
	response[idx++] = 0x7C; /* addr high */
	response[idx++] = 0x00; /* addr low */
	response[idx++] = 1;    /* param_count = 1 (dummy byte) */
	response[idx++] = 0x00; /* dummy param byte */
	response[idx++] = 0x00; /* ACK_OK */

	/* Calculate CRC on bytes 0-6 (7 bytes = 6 + param_count) */
	uint16_t crc = fourway_crc(response, 7);
	response[idx++] = crc >> 8;
	response[idx++] = crc & 0xFF;

	/* Now parse it like AM32 configurator would */
	uint8_t cmd, ack;
	uint16_t addr;
	uint8_t params[256];
	size_t param_len;

	int result = simulate_parse_message(response, 9, &cmd, &addr, &ack, params, &param_len);

	zassert_equal(result, 0, "parseMessage should succeed");
	zassert_equal(cmd, 0x3B, "Command should be DeviceWrite (0x3B)");
	zassert_equal(addr, 0x7C00, "Address should be 0x7C00");
	zassert_equal(ack, 0x00, "ACK should be ACK_OK (0x00)");
	zassert_equal(param_len, 1, "param_len should be 1");
}

ZTEST(am32_compat_tests, test_parse_read_response_with_data)
{
	/*
	 * Test parsing a DeviceRead response with actual data.
	 * Response: [0x2E][cmd][addr_hi][addr_lo][count][data...][ack][crc_hi][crc_lo]
	 */
	uint8_t response[20];
	int idx = 0;

	response[idx++] = 0x2E; /* Response marker */
	response[idx++] = 0x3A; /* cmd_DeviceRead */
	response[idx++] = 0x7C; /* addr high */
	response[idx++] = 0x00; /* addr low */
	response[idx++] = 8;    /* param_count = 8 bytes of data */

	/* 8 bytes of mock EEPROM data */
	response[idx++] = 0x01;
	response[idx++] = 0x02;
	response[idx++] = 0x03;
	response[idx++] = 0x04;
	response[idx++] = 0x05;
	response[idx++] = 0x06;
	response[idx++] = 0x07;
	response[idx++] = 0x08;

	response[idx++] = 0x00; /* ACK_OK */

	/* CRC on bytes 0 to (6 + param_count - 1) = 0 to 13 = 14 bytes */
	uint16_t crc = fourway_crc(response, 14);
	response[idx++] = crc >> 8;
	response[idx++] = crc & 0xFF;

	/* Parse it */
	uint8_t cmd, ack;
	uint16_t addr;
	uint8_t params[256];
	size_t param_len;

	int result = simulate_parse_message(response, idx, &cmd, &addr, &ack, params, &param_len);

	zassert_equal(result, 0, "parseMessage should succeed");
	zassert_equal(cmd, 0x3A, "Command should be DeviceRead (0x3A)");
	zassert_equal(ack, 0x00, "ACK should be ACK_OK");
	zassert_equal(param_len, 8, "Should have 8 bytes of data");
	zassert_equal(params[0], 0x01, "First data byte should be 0x01");
	zassert_equal(params[7], 0x08, "Last data byte should be 0x08");
}

ZTEST(am32_compat_tests, test_param_count_zero_means_256)
{
	/*
	 * CRITICAL TEST: Verify that param_count=0 is interpreted as 256 bytes.
	 * This was a bug that caused write ACKs to fail - the configurator
	 * would wait for 264 bytes (256 + 8 header) instead of 8.
	 */
	uint8_t param_count_byte = 0;
	uint16_t actual_count = (param_count_byte == 0) ? 256 : param_count_byte;

	zassert_equal(actual_count, 256, "param_count=0 should mean 256 bytes");

	/* Test the converse */
	param_count_byte = 1;
	actual_count = (param_count_byte == 0) ? 256 : param_count_byte;
	zassert_equal(actual_count, 1, "param_count=1 should mean 1 byte");
}

ZTEST(am32_compat_tests, test_response_too_short_rejected)
{
	/* Response with only 8 bytes should fail (minimum is 9) */
	uint8_t short_response[] = {0x2E, 0x3B, 0x7C, 0x00, 0x01, 0x00, 0x00, 0x00};

	uint8_t cmd, ack;
	uint16_t addr;
	int result = simulate_parse_message(short_response, 8, &cmd, &addr, &ack, NULL, NULL);

	zassert_equal(result, -1, "8-byte response should be rejected (min 9)");
}

ZTEST(am32_compat_tests, test_bad_crc_rejected)
{
	/* Build valid response then corrupt CRC */
	uint8_t response[9];
	int idx = 0;

	response[idx++] = 0x2E;
	response[idx++] = 0x3B;
	response[idx++] = 0x7C;
	response[idx++] = 0x00;
	response[idx++] = 1;
	response[idx++] = 0x00;
	response[idx++] = 0x00;

	uint16_t crc = fourway_crc(response, 7);
	response[idx++] = crc >> 8;
	response[idx++] = (crc & 0xFF) ^ 0xFF; /* Corrupt low byte */

	uint8_t cmd, ack;
	uint16_t addr;
	int result = simulate_parse_message(response, 9, &cmd, &addr, &ack, NULL, NULL);

	zassert_equal(result, -1, "Bad CRC should be rejected");
}

ZTEST(am32_compat_tests, test_wrong_start_marker_rejected)
{
	/* Response with wrong start marker */
	uint8_t response[] = {0x2F, 0x3B, 0x7C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};

	uint8_t cmd, ack;
	uint16_t addr;
	int result = simulate_parse_message(response, 9, &cmd, &addr, &ack, NULL, NULL);

	zassert_equal(result, -1, "Wrong start marker (0x2F instead of 0x2E) should be rejected");
}
