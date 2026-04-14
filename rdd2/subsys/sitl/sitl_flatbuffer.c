/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sitl_flatbuffer.h"

#include <string.h>

#include <zephyr/sys/util.h>

#define RDD2_SITL_FB_SIM_INPUT_IDENTIFIER "SYSI"

enum {
	SIM_FIELD_GYRO = 0,
	SIM_FIELD_ACCEL = 1,
	SIM_FIELD_RC = 2,
	SIM_FIELD_RC_LINK_QUALITY = 3,
	SIM_FIELD_RC_VALID = 4,
	SIM_FIELD_IMU_VALID = 5,
};

BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);
BUILD_ASSERT(sizeof(synapse_topic_Vec3f_t) == 12U);
BUILD_ASSERT(sizeof(synapse_topic_RcChannels16_t) == 64U);

static uint16_t get_le16(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint32_t get_le32(const uint8_t *buf)
{
	return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) |
	       ((uint32_t)buf[3] << 24);
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

	*field_offset = 0U;

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
	vtable_size = get_le16(vtable + 0U);
	*object_size = get_le16(vtable + 2U);

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

bool rdd2_sitl_fb_unpack_input(const uint8_t *buf, size_t buf_size, synapse_topic_Vec3f_t *gyro,
			       synapse_topic_Vec3f_t *accel, synapse_topic_RcChannels16_t *rc,
			       uint8_t *rc_link_quality, bool *rc_valid, bool *imu_valid)
{
	const uint8_t *table;
	size_t table_offset;
	uint16_t object_size = 0U;
	uint16_t field_offset = 0U;

	if (buf == NULL || buf_size < 8U) {
		return false;
	}

	if (memcmp(buf + 4, RDD2_SITL_FB_SIM_INPUT_IDENTIFIER, 4U) != 0) {
		return false;
	}

	table_offset = get_le32(buf);
	if (table_offset >= buf_size) {
		return false;
	}

	table = buf + table_offset;

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_GYRO,
					 &field_offset, &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					sizeof(synapse_topic_Vec3f_t), buf_size)) {
		return false;
	}
	if (gyro != NULL) {
		memcpy(gyro, table + field_offset, sizeof(*gyro));
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_ACCEL,
					 &field_offset, &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					sizeof(synapse_topic_Vec3f_t), buf_size)) {
		return false;
	}
	if (accel != NULL) {
		memcpy(accel, table + field_offset, sizeof(*accel));
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC, &field_offset,
					 &object_size) ||
	    !sim_input_struct_in_bounds(table_offset, object_size, field_offset,
					sizeof(synapse_topic_RcChannels16_t), buf_size)) {
		return false;
	}
	if (rc != NULL) {
		memcpy(rc, table + field_offset, sizeof(*rc));
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC_LINK_QUALITY,
					 &field_offset, &object_size)) {
		return false;
	}
	if (rc_link_quality != NULL) {
		*rc_link_quality = 0U;
		if (field_offset != 0U) {
			if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U,
							buf_size)) {
				return false;
			}
			*rc_link_quality = table[field_offset];
		}
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_RC_VALID,
					 &field_offset, &object_size)) {
		return false;
	}
	if (rc_valid != NULL) {
		*rc_valid = false;
		if (field_offset != 0U) {
			if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U,
							buf_size)) {
				return false;
			}
			*rc_valid = table[field_offset] != 0U;
		}
	}

	if (!sim_input_read_field_offset(table, table_offset, buf_size, SIM_FIELD_IMU_VALID,
					 &field_offset, &object_size)) {
		return false;
	}
	if (imu_valid != NULL) {
		*imu_valid = false;
		if (field_offset != 0U) {
			if (!sim_input_struct_in_bounds(table_offset, object_size, field_offset, 1U,
							buf_size)) {
				return false;
			}
			*imu_valid = table[field_offset] != 0U;
		}
	}

	return true;
}
