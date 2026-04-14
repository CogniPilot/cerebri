/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "topic_bus.h"

#include <zephyr/sys/atomic.h>

#include <zros/private/zros_topic_struct.h>

ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(rc, synapse_topic_RcChannels16_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(flight_state, rdd2_topic_flight_state_blob_t);
ZROS_TOPIC_DEFINE_SINGLE_PUBLISHER(motor_output, rdd2_topic_motor_output_blob_t);

uint32_t rdd2_topic_generation(const struct zros_topic *topic)
{
	return (uint32_t)atomic_get((atomic_t *)&topic->_lockless_generation);
}

bool rdd2_topic_has_sample(const struct zros_topic *topic)
{
	return rdd2_topic_generation(topic) != 0U;
}

bool rdd2_topic_copy_blob(const struct zros_topic *topic, uint8_t *buf, size_t buf_size,
			  size_t *len)
{
	if (topic == NULL || buf == NULL || len == NULL) {
		return false;
	}

	if (!rdd2_topic_has_sample(topic) || (size_t)topic->_size > buf_size) {
		return false;
	}

	if (zros_topic_read((struct zros_topic *)topic, buf) != 0) {
		return false;
	}

	*len = (size_t)topic->_size;
	return true;
}

uint32_t rdd2_topic_flight_state_generation(void)
{
	return rdd2_topic_generation(&topic_flight_state);
}

bool rdd2_topic_flight_state_copy_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	return rdd2_topic_copy_blob(&topic_flight_state, buf, buf_size, len);
}

uint32_t rdd2_topic_motor_output_generation(void)
{
	return rdd2_topic_generation(&topic_motor_output);
}

bool rdd2_topic_motor_output_copy_blob(uint8_t *buf, size_t buf_size, size_t *len)
{
	return rdd2_topic_copy_blob(&topic_motor_output, buf, buf_size, len);
}
