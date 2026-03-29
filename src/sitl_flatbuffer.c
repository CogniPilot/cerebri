/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sitl_flatbuffer.h"

#include <string.h>

enum {
	SIM_FIELD_GYRO = 0,
	SIM_FIELD_ACCEL = 1,
	SIM_FIELD_RC = 2,
	SIM_FIELD_RC_LINK_QUALITY = 3,
	SIM_FIELD_RC_VALID = 4,
	SIM_FIELD_IMU_VALID = 5,
};

static uint16_t get_le16(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint32_t get_le32(const uint8_t *buf)
{
	return (uint32_t)buf[0] |
	       ((uint32_t)buf[1] << 8) |
	       ((uint32_t)buf[2] << 16) |
	       ((uint32_t)buf[3] << 24);
}

static float get_float_le(const uint8_t *buf)
{
	uint32_t raw = get_le32(buf);
	float value = 0.0f;

	memcpy(&value, &raw, sizeof(value));

	return value;
}

static bool sim_input_read_field_offset(const uint8_t *table, size_t table_offset, size_t buf_size,
					uint16_t field_index, uint16_t *field_offset,
					uint16_t *object_size)
{
	size_t entry_offset = 4U + ((size_t)field_index * sizeof(uint16_t));
	uint32_t vtable_distance;
	size_t vtable_offset;
	const uint8_t *vtable;
	uint16_t vtable_size;

	*field_offset = 0;

	if ((table_offset + sizeof(uint32_t)) > buf_size) {
		return false;
	}

	vtable_distance = get_le32(table);
	if (vtable_distance > table_offset) {
		return false;
	}

	vtable_offset = table_offset - vtable_distance;
	if ((vtable_offset + 4U) > buf_size) {
		return false;
	}

	vtable = table - vtable_distance;
	vtable_size = get_le16(vtable + 0);
	*object_size = get_le16(vtable + 2);

	if ((vtable_offset + vtable_size) > buf_size) {
		return false;
	}

	if (entry_offset >= vtable_size) {
		return true;
	}

	*field_offset = get_le16(vtable + entry_offset);

	return true;
}

static bool sim_input_struct_in_bounds(size_t table_offset, uint16_t object_size,
				       uint16_t field_offset, size_t field_size, size_t buf_size)
{
	if (field_offset == 0U || field_offset >= object_size) {
		return false;
	}

	return (table_offset + (size_t)field_offset + field_size) <= buf_size;
}

static void unpack_vec3f(const uint8_t *buf, float values[3])
{
	for (size_t i = 0; i < 3U; i++) {
		values[i] = get_float_le(buf + (i * sizeof(uint32_t)));
	}
}

static void unpack_rc_channels(const uint8_t *buf, struct rc_channels_msg *rc)
{
	for (size_t i = 0; i < 16U; i++) {
		rc->us[i] = (int32_t)get_le32(buf + (i * sizeof(uint32_t)));
	}
}

bool cerebri2_sitl_fb_unpack_input(
	const uint8_t *buf, size_t buf_size, struct sitl_input_msg *input)
{
	const uint8_t *table;
	size_t table_offset;
	uint16_t object_size = 0U;
	uint16_t field_offset = 0U;

	if (buf == NULL || input == NULL || buf_size < 8U) {
		return false;
	}

	if (memcmp(buf + 4, CEREBRI2_SITL_FB_SIM_INPUT_IDENTIFIER, 4U) != 0) {
		return false;
	}

	table_offset = get_le32(buf);
	if (table_offset >= buf_size) {
		return false;
	}

	table = buf + table_offset;
	memset(input, 0, sizeof(*input));

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_GYRO,
					 &field_offset, &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					3U * sizeof(uint32_t), buf_size)) {
		return false;
	}
	unpack_vec3f(table + field_offset, input->imu.gyro_rad_s);

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_ACCEL,
					 &field_offset, &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					3U * sizeof(uint32_t), buf_size)) {
		return false;
	}
	unpack_vec3f(table + field_offset, input->imu.accel_m_s2);

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC,
					 &field_offset, &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					16U * sizeof(uint32_t), buf_size)) {
		return false;
	}
	unpack_rc_channels(table + field_offset, &input->rc);

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC_LINK_QUALITY,
					 &field_offset, &object_size)) {
		return false;
	}
	if (field_offset != 0U) {
		if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U, buf_size)) {
			return false;
		}
		input->rc_link_quality = table[field_offset];
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC_VALID,
					 &field_offset, &object_size)) {
		return false;
	}
	if (field_offset != 0U) {
		if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U, buf_size)) {
			return false;
		}
		input->rc_valid = table[field_offset] != 0U;
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_IMU_VALID,
					 &field_offset, &object_size)) {
		return false;
	}
	if (field_offset != 0U) {
		if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U, buf_size)) {
			return false;
		}
		input->imu_valid = table[field_offset] != 0U;
	}

	return true;
}
