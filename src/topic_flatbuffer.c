/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_flatbuffer.h"

#include <string.h>

#include <zephyr/sys/util.h>

enum {
	FLIGHT_VTABLE_OFFSET = 4,
	FLIGHT_VTABLE_SIZE = 16,
	FLIGHT_TABLE_OFFSET = FLIGHT_VTABLE_OFFSET + FLIGHT_VTABLE_SIZE,
	FLIGHT_OBJECT_SIZE = 144,
	FLIGHT_FIELD_GYRO = 4,
	FLIGHT_FIELD_ACCEL = 16,
	FLIGHT_FIELD_RC = 28,
	FLIGHT_FIELD_STATUS = 96,
	FLIGHT_FIELD_RATE_DESIRED = 120,
	FLIGHT_FIELD_RATE_CMD = 132,
};

enum {
	MOTOR_VTABLE_OFFSET = 4,
	MOTOR_VTABLE_SIZE = 12,
	MOTOR_TABLE_OFFSET = MOTOR_VTABLE_OFFSET + MOTOR_VTABLE_SIZE,
	MOTOR_OBJECT_SIZE = 32,
	MOTOR_FIELD_MOTORS = 4,
	MOTOR_FIELD_RAW = 20,
	MOTOR_FIELD_ARMED = 28,
	MOTOR_FIELD_TEST_MODE = 29,
};

BUILD_ASSERT(sizeof(cerebri_topic_Vec3f_t) == 12U);
BUILD_ASSERT(sizeof(cerebri_topic_RcChannels16_t) == 64U);
BUILD_ASSERT(sizeof(cerebri_topic_ControlStatus_t) == 24U);
BUILD_ASSERT(sizeof(cerebri_topic_RateTriplet_t) == 12U);
BUILD_ASSERT(sizeof(cerebri_topic_MotorValues4f_t) == 16U);
BUILD_ASSERT(sizeof(cerebri_topic_MotorRaw4u16_t) == 8U);
BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

static void put_le16(uint16_t value, uint8_t *buf)
{
	buf[0] = (uint8_t)(value & 0xffU);
	buf[1] = (uint8_t)((value >> 8) & 0xffU);
}

static uint16_t get_le16(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static void put_le32(uint32_t value, uint8_t *buf)
{
	buf[0] = (uint8_t)(value & 0xffU);
	buf[1] = (uint8_t)((value >> 8) & 0xffU);
	buf[2] = (uint8_t)((value >> 16) & 0xffU);
	buf[3] = (uint8_t)((value >> 24) & 0xffU);
}

static uint32_t get_le32(const uint8_t *buf)
{
	return (uint32_t)buf[0] |
	       ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16) |
	       ((uint32_t)buf[3] << 24);
}

static size_t pack_fixed_table(
	uint8_t *buf, size_t buf_size, size_t total_size, uint16_t vtable_size, uint16_t object_size,
	const uint16_t *field_offsets, size_t field_count)
{
	uint8_t *table;
	uint8_t *vtable;

	if (buf == NULL || field_offsets == NULL || buf_size < total_size) {
		return 0U;
	}

	memset(buf, 0, total_size);
	put_le32((uint32_t)(4U + vtable_size), buf);

	vtable = buf + 4U;
	put_le16(vtable_size, vtable + 0U);
	put_le16(object_size, vtable + 2U);
	for (size_t i = 0; i < field_count; i++) {
		put_le16(field_offsets[i], vtable + 4U + (uint16_t)(i * sizeof(uint16_t)));
	}

	table = buf + 4U + vtable_size;
	put_le32((uint32_t)vtable_size, table);

	return total_size;
}

static cerebri_topic_FlightSnapshot_table_t flight_state_root(
	const uint8_t *buf, size_t buf_size)
{
	const uint8_t *vtable;
	const uint8_t *table;
	uint32_t root_offset;
	uint32_t vtable_distance;

	if (buf_size != CEREBRI_TOPIC_FB_FLIGHT_STATE_SIZE) {
		return NULL;
	}

	root_offset = get_le32(buf);
	if (root_offset != FLIGHT_TABLE_OFFSET) {
		return NULL;
	}

	table = buf + root_offset;
	vtable_distance = get_le32(table);
	if (vtable_distance != (FLIGHT_TABLE_OFFSET - FLIGHT_VTABLE_OFFSET)) {
		return NULL;
	}

	vtable = table - vtable_distance;
	if (get_le16(vtable + 0) != FLIGHT_VTABLE_SIZE ||
	    get_le16(vtable + 2) != FLIGHT_OBJECT_SIZE ||
	    get_le16(vtable + 4) != FLIGHT_FIELD_GYRO ||
	    get_le16(vtable + 6) != FLIGHT_FIELD_ACCEL ||
	    get_le16(vtable + 8) != FLIGHT_FIELD_RC ||
	    get_le16(vtable + 10) != FLIGHT_FIELD_STATUS ||
	    get_le16(vtable + 12) != FLIGHT_FIELD_RATE_DESIRED ||
	    get_le16(vtable + 14) != FLIGHT_FIELD_RATE_CMD) {
		return NULL;
	}

	return cerebri_topic_FlightSnapshot_as_root(buf);
}

static cerebri_topic_MotorOutput_table_t motor_output_root(const uint8_t *buf, size_t buf_size)
{
	const uint8_t *vtable;
	const uint8_t *table;
	uint32_t root_offset;
	uint32_t vtable_distance;

	if (buf_size != CEREBRI_TOPIC_FB_MOTOR_OUTPUT_SIZE) {
		return NULL;
	}

	root_offset = get_le32(buf);
	if (root_offset != MOTOR_TABLE_OFFSET) {
		return NULL;
	}

	table = buf + root_offset;
	vtable_distance = get_le32(table);
	if (vtable_distance != (MOTOR_TABLE_OFFSET - MOTOR_VTABLE_OFFSET)) {
		return NULL;
	}

	vtable = table - vtable_distance;
	if (get_le16(vtable + 0) != MOTOR_VTABLE_SIZE ||
	    get_le16(vtable + 2) != MOTOR_OBJECT_SIZE ||
	    get_le16(vtable + 4) != MOTOR_FIELD_MOTORS ||
	    get_le16(vtable + 6) != MOTOR_FIELD_RAW ||
	    get_le16(vtable + 8) != MOTOR_FIELD_ARMED ||
	    get_le16(vtable + 10) != MOTOR_FIELD_TEST_MODE) {
		return NULL;
	}

	return cerebri_topic_MotorOutput_as_root(buf);
}

size_t cerebri_topic_fb_pack_flight_state(
	uint8_t *buf, size_t buf_size, const cerebri_topic_Vec3f_t *gyro,
	const cerebri_topic_Vec3f_t *accel, const cerebri_topic_RcChannels16_t *rc,
	const cerebri_topic_ControlStatus_t *status,
	const cerebri_topic_RateTriplet_t *rate_desired,
	const cerebri_topic_RateTriplet_t *rate_cmd)
{
	static const uint16_t field_offsets[] = {
		FLIGHT_FIELD_GYRO,
		FLIGHT_FIELD_ACCEL,
		FLIGHT_FIELD_RC,
		FLIGHT_FIELD_STATUS,
		FLIGHT_FIELD_RATE_DESIRED,
		FLIGHT_FIELD_RATE_CMD,
	};
	uint8_t *table;
	size_t length;

	if (gyro == NULL || accel == NULL || rc == NULL || status == NULL || rate_desired == NULL ||
	    rate_cmd == NULL) {
		return 0U;
	}

	length = pack_fixed_table(buf, buf_size, CEREBRI_TOPIC_FB_FLIGHT_STATE_SIZE,
				  FLIGHT_VTABLE_SIZE, FLIGHT_OBJECT_SIZE, field_offsets,
				  ARRAY_SIZE(field_offsets));
	if (length == 0U) {
		return 0U;
	}

	table = buf + FLIGHT_TABLE_OFFSET;
	memcpy(table + FLIGHT_FIELD_GYRO, gyro, sizeof(*gyro));
	memcpy(table + FLIGHT_FIELD_ACCEL, accel, sizeof(*accel));
	memcpy(table + FLIGHT_FIELD_RC, rc, sizeof(*rc));
	memcpy(table + FLIGHT_FIELD_STATUS, status, sizeof(*status));
	memcpy(table + FLIGHT_FIELD_RATE_DESIRED, rate_desired, sizeof(*rate_desired));
	memcpy(table + FLIGHT_FIELD_RATE_CMD, rate_cmd, sizeof(*rate_cmd));

	return length;
}

bool cerebri_topic_fb_unpack_flight_state(
	const uint8_t *buf, size_t buf_size, cerebri_topic_Vec3f_t *gyro,
	cerebri_topic_Vec3f_t *accel, cerebri_topic_RcChannels16_t *rc,
	cerebri_topic_ControlStatus_t *status, cerebri_topic_RateTriplet_t *rate_desired,
	cerebri_topic_RateTriplet_t *rate_cmd)
{
	cerebri_topic_FlightSnapshot_table_t flight_state = flight_state_root(buf, buf_size);

	if (flight_state == NULL || gyro == NULL || accel == NULL || rc == NULL ||
	    status == NULL ||
	    rate_desired == NULL || rate_cmd == NULL) {
		return false;
	}

	memcpy(gyro, cerebri_topic_FlightSnapshot_gyro(flight_state), sizeof(*gyro));
	memcpy(accel, cerebri_topic_FlightSnapshot_accel(flight_state), sizeof(*accel));
	memcpy(rc, cerebri_topic_FlightSnapshot_rc(flight_state), sizeof(*rc));
	memcpy(status, cerebri_topic_FlightSnapshot_status(flight_state), sizeof(*status));
	memcpy(rate_desired, cerebri_topic_FlightSnapshot_rate_desired(flight_state),
	       sizeof(*rate_desired));
	memcpy(rate_cmd, cerebri_topic_FlightSnapshot_rate_cmd(flight_state), sizeof(*rate_cmd));

	return true;
}

size_t cerebri_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const cerebri_topic_MotorValues4f_t *motors,
	const cerebri_topic_MotorRaw4u16_t *raw, bool armed, bool test_mode)
{
	static const uint16_t field_offsets[] = {
		MOTOR_FIELD_MOTORS,
		MOTOR_FIELD_RAW,
		MOTOR_FIELD_ARMED,
		MOTOR_FIELD_TEST_MODE,
	};
	uint8_t *table;
	size_t length;

	if (motors == NULL || raw == NULL) {
		return 0U;
	}

	length = pack_fixed_table(buf, buf_size, CEREBRI_TOPIC_FB_MOTOR_OUTPUT_SIZE,
				  MOTOR_VTABLE_SIZE, MOTOR_OBJECT_SIZE, field_offsets,
				  ARRAY_SIZE(field_offsets));
	if (length == 0U) {
		return 0U;
	}

	table = buf + MOTOR_TABLE_OFFSET;
	memcpy(table + MOTOR_FIELD_MOTORS, motors, sizeof(*motors));
	memcpy(table + MOTOR_FIELD_RAW, raw, sizeof(*raw));
	table[MOTOR_FIELD_ARMED] = armed ? 1U : 0U;
	table[MOTOR_FIELD_TEST_MODE] = test_mode ? 1U : 0U;

	return length;
}

bool cerebri_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, cerebri_topic_MotorValues4f_t *motors,
	cerebri_topic_MotorRaw4u16_t *raw, bool *armed, bool *test_mode)
{
	cerebri_topic_MotorOutput_table_t output = motor_output_root(buf, buf_size);

	if (output == NULL || motors == NULL || raw == NULL || armed == NULL || test_mode == NULL) {
		return false;
	}

	memcpy(motors, cerebri_topic_MotorOutput_motors(output), sizeof(*motors));
	memcpy(raw, cerebri_topic_MotorOutput_raw(output), sizeof(*raw));
	*armed = cerebri_topic_MotorOutput_armed(output) != 0U;
	*test_mode = cerebri_topic_MotorOutput_test_mode(output) != 0U;

	return true;
}
