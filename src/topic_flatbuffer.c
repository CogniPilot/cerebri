/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_flatbuffer.h"

#include <string.h>

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
	FLIGHT_STATUS_RC_STAMP_MS = 0,
	FLIGHT_STATUS_THROTTLE_US = 8,
	FLIGHT_STATUS_RC_LINK_QUALITY = 12,
	FLIGHT_STATUS_ARMED = 13,
	FLIGHT_STATUS_RC_VALID = 14,
	FLIGHT_STATUS_RC_STALE = 15,
	FLIGHT_STATUS_IMU_OK = 16,
	FLIGHT_STATUS_ARM_SWITCH = 17,
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

static void put_le64(uint64_t value, uint8_t *buf)
{
	put_le32((uint32_t)(value & 0xffffffffULL), buf + 0);
	put_le32((uint32_t)(value >> 32), buf + 4);
}

static uint64_t get_le64(const uint8_t *buf)
{
	return (uint64_t)get_le32(buf + 0) | ((uint64_t)get_le32(buf + 4) << 32);
}

static void put_float_le(uint8_t *buf, float value)
{
	uint32_t raw = 0U;

	memcpy(&raw, &value, sizeof(raw));
	put_le32(raw, buf);
}

static float get_float_le(const uint8_t *buf)
{
	uint32_t raw = get_le32(buf);
	float value = 0.0f;

	memcpy(&value, &raw, sizeof(value));

	return value;
}

static void pack_vec3f(uint8_t *buf, const float values[3])
{
	for (size_t i = 0; i < 3; i++) {
		put_float_le(buf + (i * sizeof(uint32_t)), values[i]);
	}
}

static void unpack_vec3f(const uint8_t *buf, float values[3])
{
	for (size_t i = 0; i < 3; i++) {
		values[i] = get_float_le(buf + (i * sizeof(uint32_t)));
	}
}

static void pack_rate_triplet(uint8_t *buf, const struct rate_triplet *rate)
{
	put_float_le(buf + 0, rate->roll);
	put_float_le(buf + 4, rate->pitch);
	put_float_le(buf + 8, rate->yaw);
}

static void unpack_rate_triplet(const uint8_t *buf, struct rate_triplet *rate)
{
	rate->roll = get_float_le(buf + 0);
	rate->pitch = get_float_le(buf + 4);
	rate->yaw = get_float_le(buf + 8);
}

static void pack_rc_channels(uint8_t *buf, const struct rc_channels_msg *rc)
{
	for (size_t i = 0; i < 16U; i++) {
		put_le32((uint32_t)rc->us[i], buf + (i * sizeof(uint32_t)));
	}
}

static void unpack_rc_channels(const uint8_t *buf, struct rc_channels_msg *rc)
{
	for (size_t i = 0; i < 16U; i++) {
		rc->us[i] = (int32_t)get_le32(buf + (i * sizeof(uint32_t)));
	}
}

static void pack_status(uint8_t *buf, const struct control_status_msg *status)
{
	put_le64((uint64_t)status->rc_stamp_ms, buf + FLIGHT_STATUS_RC_STAMP_MS);
	put_le32((uint32_t)status->throttle_us, buf + FLIGHT_STATUS_THROTTLE_US);
	buf[FLIGHT_STATUS_RC_LINK_QUALITY] = status->rc_link_quality;
	buf[FLIGHT_STATUS_ARMED] = status->armed ? 1U : 0U;
	buf[FLIGHT_STATUS_RC_VALID] = status->rc_valid ? 1U : 0U;
	buf[FLIGHT_STATUS_RC_STALE] = status->rc_stale ? 1U : 0U;
	buf[FLIGHT_STATUS_IMU_OK] = status->imu_ok ? 1U : 0U;
	buf[FLIGHT_STATUS_ARM_SWITCH] = status->arm_switch ? 1U : 0U;
}

static void unpack_status(const uint8_t *buf, struct control_status_msg *status)
{
	status->rc_stamp_ms = (int64_t)get_le64(buf + FLIGHT_STATUS_RC_STAMP_MS);
	status->throttle_us = (int32_t)get_le32(buf + FLIGHT_STATUS_THROTTLE_US);
	status->rc_link_quality = buf[FLIGHT_STATUS_RC_LINK_QUALITY];
	status->armed = buf[FLIGHT_STATUS_ARMED] != 0U;
	status->rc_valid = buf[FLIGHT_STATUS_RC_VALID] != 0U;
	status->rc_stale = buf[FLIGHT_STATUS_RC_STALE] != 0U;
	status->imu_ok = buf[FLIGHT_STATUS_IMU_OK] != 0U;
	status->arm_switch = buf[FLIGHT_STATUS_ARM_SWITCH] != 0U;
}

static void pack_motor_values(uint8_t *buf, const float motors[4])
{
	for (size_t i = 0; i < 4; i++) {
		put_float_le(buf + (i * sizeof(uint32_t)), motors[i]);
	}
}

static void unpack_motor_values(const uint8_t *buf, float motors[4])
{
	for (size_t i = 0; i < 4; i++) {
		motors[i] = get_float_le(buf + (i * sizeof(uint32_t)));
	}
}

static void pack_motor_raw(uint8_t *buf, const uint16_t raw[4])
{
	for (size_t i = 0; i < 4; i++) {
		put_le16(raw[i], buf + (i * sizeof(uint16_t)));
	}
}

static void unpack_motor_raw(const uint8_t *buf, uint16_t raw[4])
{
	for (size_t i = 0; i < 4; i++) {
		raw[i] = get_le16(buf + (i * sizeof(uint16_t)));
	}
}

static bool flight_snapshot_layout_valid(const uint8_t *buf, size_t buf_size)
{
	const uint8_t *vtable;
	const uint8_t *table;
	uint32_t root_offset;
	uint32_t vtable_distance;

	if (buf_size != CEREBRI2_TOPIC_FB_FLIGHT_SNAPSHOT_SIZE) {
		return false;
	}

	root_offset = get_le32(buf);
	if (root_offset != FLIGHT_TABLE_OFFSET) {
		return false;
	}

	table = buf + root_offset;
	vtable_distance = get_le32(table);
	if (vtable_distance != (FLIGHT_TABLE_OFFSET - FLIGHT_VTABLE_OFFSET)) {
		return false;
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
		return false;
	}

	return true;
}

static bool motor_output_layout_valid(const uint8_t *buf, size_t buf_size)
{
	const uint8_t *vtable;
	const uint8_t *table;
	uint32_t root_offset;
	uint32_t vtable_distance;

	if (buf_size != CEREBRI2_TOPIC_FB_MOTOR_OUTPUT_SIZE) {
		return false;
	}

	root_offset = get_le32(buf);
	if (root_offset != MOTOR_TABLE_OFFSET) {
		return false;
	}

	table = buf + root_offset;
	vtable_distance = get_le32(table);
	if (vtable_distance != (MOTOR_TABLE_OFFSET - MOTOR_VTABLE_OFFSET)) {
		return false;
	}

	vtable = table - vtable_distance;
	if (get_le16(vtable + 0) != MOTOR_VTABLE_SIZE ||
	    get_le16(vtable + 2) != MOTOR_OBJECT_SIZE ||
	    get_le16(vtable + 4) != MOTOR_FIELD_MOTORS ||
	    get_le16(vtable + 6) != MOTOR_FIELD_RAW ||
	    get_le16(vtable + 8) != MOTOR_FIELD_ARMED ||
	    get_le16(vtable + 10) != MOTOR_FIELD_TEST_MODE) {
		return false;
	}

	return true;
}

size_t cerebri2_topic_fb_pack_flight_snapshot(
	uint8_t *buf, size_t buf_size, const struct flight_snapshot *snapshot)
{
	uint8_t *table;
	uint8_t *vtable;

	if (buf == NULL || snapshot == NULL ||
	    buf_size < CEREBRI2_TOPIC_FB_FLIGHT_SNAPSHOT_SIZE) {
		return 0U;
	}

	memset(buf, 0, CEREBRI2_TOPIC_FB_FLIGHT_SNAPSHOT_SIZE);
	put_le32(FLIGHT_TABLE_OFFSET, buf);

	vtable = buf + FLIGHT_VTABLE_OFFSET;
	put_le16(FLIGHT_VTABLE_SIZE, vtable + 0);
	put_le16(FLIGHT_OBJECT_SIZE, vtable + 2);
	put_le16(FLIGHT_FIELD_GYRO, vtable + 4);
	put_le16(FLIGHT_FIELD_ACCEL, vtable + 6);
	put_le16(FLIGHT_FIELD_RC, vtable + 8);
	put_le16(FLIGHT_FIELD_STATUS, vtable + 10);
	put_le16(FLIGHT_FIELD_RATE_DESIRED, vtable + 12);
	put_le16(FLIGHT_FIELD_RATE_CMD, vtable + 14);

	table = buf + FLIGHT_TABLE_OFFSET;
	put_le32(FLIGHT_TABLE_OFFSET - FLIGHT_VTABLE_OFFSET, table);
	pack_vec3f(table + FLIGHT_FIELD_GYRO, snapshot->imu.gyro_rad_s);
	pack_vec3f(table + FLIGHT_FIELD_ACCEL, snapshot->imu.accel_m_s2);
	pack_rc_channels(table + FLIGHT_FIELD_RC, &snapshot->rc);
	pack_status(table + FLIGHT_FIELD_STATUS, &snapshot->status);
	pack_rate_triplet(table + FLIGHT_FIELD_RATE_DESIRED, &snapshot->rate.desired);
	pack_rate_triplet(table + FLIGHT_FIELD_RATE_CMD, &snapshot->rate.cmd);

	return CEREBRI2_TOPIC_FB_FLIGHT_SNAPSHOT_SIZE;
}

bool cerebri2_topic_fb_unpack_flight_snapshot(
	const uint8_t *buf, size_t buf_size, struct flight_snapshot *snapshot)
{
	const uint8_t *table;

	if (snapshot == NULL || !flight_snapshot_layout_valid(buf, buf_size)) {
		return false;
	}

	memset(snapshot, 0, sizeof(*snapshot));
	table = buf + FLIGHT_TABLE_OFFSET;
	unpack_vec3f(table + FLIGHT_FIELD_GYRO, snapshot->imu.gyro_rad_s);
	unpack_vec3f(table + FLIGHT_FIELD_ACCEL, snapshot->imu.accel_m_s2);
	unpack_rc_channels(table + FLIGHT_FIELD_RC, &snapshot->rc);
	unpack_status(table + FLIGHT_FIELD_STATUS, &snapshot->status);
	unpack_rate_triplet(table + FLIGHT_FIELD_RATE_DESIRED, &snapshot->rate.desired);
	unpack_rate_triplet(table + FLIGHT_FIELD_RATE_CMD, &snapshot->rate.cmd);

	return true;
}

size_t cerebri2_topic_fb_pack_motor_output(
	uint8_t *buf, size_t buf_size, const struct motor_output_msg *output)
{
	uint8_t *table;
	uint8_t *vtable;

	if (buf == NULL || output == NULL || buf_size < CEREBRI2_TOPIC_FB_MOTOR_OUTPUT_SIZE) {
		return 0U;
	}

	memset(buf, 0, CEREBRI2_TOPIC_FB_MOTOR_OUTPUT_SIZE);
	put_le32(MOTOR_TABLE_OFFSET, buf);

	vtable = buf + MOTOR_VTABLE_OFFSET;
	put_le16(MOTOR_VTABLE_SIZE, vtable + 0);
	put_le16(MOTOR_OBJECT_SIZE, vtable + 2);
	put_le16(MOTOR_FIELD_MOTORS, vtable + 4);
	put_le16(MOTOR_FIELD_RAW, vtable + 6);
	put_le16(MOTOR_FIELD_ARMED, vtable + 8);
	put_le16(MOTOR_FIELD_TEST_MODE, vtable + 10);

	table = buf + MOTOR_TABLE_OFFSET;
	put_le32(MOTOR_TABLE_OFFSET - MOTOR_VTABLE_OFFSET, table);
	pack_motor_values(table + MOTOR_FIELD_MOTORS, output->motors);
	pack_motor_raw(table + MOTOR_FIELD_RAW, output->raw);
	table[MOTOR_FIELD_ARMED] = output->armed ? 1U : 0U;
	table[MOTOR_FIELD_TEST_MODE] = output->test_mode ? 1U : 0U;

	return CEREBRI2_TOPIC_FB_MOTOR_OUTPUT_SIZE;
}

bool cerebri2_topic_fb_unpack_motor_output(
	const uint8_t *buf, size_t buf_size, struct motor_output_msg *output)
{
	const uint8_t *table;

	if (output == NULL || !motor_output_layout_valid(buf, buf_size)) {
		return false;
	}

	memset(output, 0, sizeof(*output));
	table = buf + MOTOR_TABLE_OFFSET;
	unpack_motor_values(table + MOTOR_FIELD_MOTORS, output->motors);
	unpack_motor_raw(table + MOTOR_FIELD_RAW, output->raw);
	output->armed = table[MOTOR_FIELD_ARMED] != 0U;
	output->test_mode = table[MOTOR_FIELD_TEST_MODE] != 0U;

	return true;
}
