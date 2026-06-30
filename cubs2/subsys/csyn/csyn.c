/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "csyn.h"

#include <string.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
#include "control_io.h"
#include "host_deploy_io.h"
#endif

struct csyn_topic_store {
	uint8_t slots[2][CUBS2_CSYN_TOPIC_MAX_SIZE];
	uint16_t lengths[2];
	atomic_t generation;
};

static struct csyn_topic_store g_csyn_topics[CUBS2_CSYN_TOPIC_COUNT];

static const struct cubs2_csyn_topic_info g_csyn_topic_info[] = {
	[CUBS2_CSYN_TOPIC_FLIGHT_SNAPSHOT] = {
		.name = "flight_snapshot",
		.keyexpr = "synapse/flight_snapshot",
		.type_name = "synapse.topic.FlightSnapshot",
		.kind = CUBS2_CSYN_KIND_FLATBUFFER,
		.max_size = CUBS2_TOPIC_FB_FLIGHT_STATE_SIZE,
	},
	[CUBS2_CSYN_TOPIC_MOTOR_OUTPUT] = {
		.name = "motor_output",
		.keyexpr = "synapse/motor_output",
		.type_name = "synapse.topic.MotorOutput",
		.kind = CUBS2_CSYN_KIND_FLATBUFFER,
		.max_size = CUBS2_TOPIC_FB_MOTOR_OUTPUT_SIZE,
	},
	[CUBS2_CSYN_TOPIC_CONTROL_OUTPUT] = {
		.name = "control_output",
		.keyexpr = "synapse/control_output",
		.type_name = "synapse.topic.RcChannels16",
		.kind = CUBS2_CSYN_KIND_STRUCT,
		.max_size = sizeof(synapse_topic_RcChannels16_t),
	},
	[CUBS2_CSYN_TOPIC_MOCAP_FRAME] = {
		.name = "mocap_frame",
		.keyexpr = "synapse/mocap/frame",
		.type_name = "synapse.topic.MocapFrame",
		.kind = CUBS2_CSYN_KIND_FLATBUFFER,
		.max_size = CUBS2_TOPIC_FB_MOCAP_FRAME_MAX_SIZE,
	},
	[CUBS2_CSYN_TOPIC_SIM_INPUT] = {
		.name = "sim_input",
		.keyexpr = "synapse/sim_input",
		.type_name = "synapse.sil.SimInput",
		.kind = CUBS2_CSYN_KIND_FLATBUFFER,
		.max_size = 256U,
	},
	[CUBS2_CSYN_TOPIC_MANUAL_CONTROL] = {
		.name = "manual_control",
		.keyexpr = "synapse/manual_control",
		.type_name = "synapse.topic.ManualControl",
		.kind = CUBS2_CSYN_KIND_OPAQUE,
		.max_size = 128U,
	},
};

BUILD_ASSERT(ARRAY_SIZE(g_csyn_topic_info) == CUBS2_CSYN_TOPIC_COUNT);
BUILD_ASSERT(CUBS2_TOPIC_FB_FLIGHT_STATE_SIZE <= CUBS2_CSYN_TOPIC_MAX_SIZE);
BUILD_ASSERT(CUBS2_TOPIC_FB_MOTOR_OUTPUT_SIZE <= CUBS2_CSYN_TOPIC_MAX_SIZE);

static bool topic_valid(enum cubs2_csyn_topic_id topic)
{
	return topic >= 0 && topic < CUBS2_CSYN_TOPIC_COUNT;
}

enum cubs2_csyn_topic_id cubs2_csyn_topic_parse(const char *name)
{
	if (name == NULL) {
		return CUBS2_CSYN_TOPIC_INVALID;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(g_csyn_topic_info); i++) {
		if (strcmp(name, g_csyn_topic_info[i].name) == 0 ||
		    strcmp(name, g_csyn_topic_info[i].keyexpr) == 0) {
			return (enum cubs2_csyn_topic_id)i;
		}
	}

	if (strcmp(name, "flight") == 0) {
		return CUBS2_CSYN_TOPIC_FLIGHT_SNAPSHOT;
	}
	if (strcmp(name, "motor") == 0) {
		return CUBS2_CSYN_TOPIC_MOTOR_OUTPUT;
	}
	if (strcmp(name, "control") == 0 || strcmp(name, "output") == 0 ||
	    strcmp(name, "actuator") == 0 || strcmp(name, "rc") == 0) {
		return CUBS2_CSYN_TOPIC_CONTROL_OUTPUT;
	}
	if (strcmp(name, "mocap") == 0) {
		return CUBS2_CSYN_TOPIC_MOCAP_FRAME;
	}

	return CUBS2_CSYN_TOPIC_INVALID;
}

const struct cubs2_csyn_topic_info *cubs2_csyn_topic_info(enum cubs2_csyn_topic_id topic)
{
	if (!topic_valid(topic)) {
		return NULL;
	}

	return &g_csyn_topic_info[topic];
}

size_t cubs2_csyn_topic_count(void)
{
	return ARRAY_SIZE(g_csyn_topic_info);
}

bool cubs2_csyn_topic_publish(enum cubs2_csyn_topic_id topic, const void *buf, size_t len)
{
	const struct cubs2_csyn_topic_info *info = cubs2_csyn_topic_info(topic);
	struct csyn_topic_store *store;
	uint32_t next_generation;
	uint32_t slot;

	if (info == NULL || buf == NULL || len == 0U || len > info->max_size ||
	    len > CUBS2_CSYN_TOPIC_MAX_SIZE) {
		return false;
	}

	store = &g_csyn_topics[topic];
	next_generation = (uint32_t)atomic_get(&store->generation) + 1U;
	slot = next_generation & 1U;

	memcpy(store->slots[slot], buf, len);
	store->lengths[slot] = (uint16_t)len;
	atomic_set(&store->generation, (atomic_val_t)next_generation);

	return true;
}

bool cubs2_csyn_topic_copy(enum cubs2_csyn_topic_id topic, void *buf, size_t buf_size,
			   size_t *len, uint32_t *generation)
{
	struct csyn_topic_store *store;
	uint32_t generation_start;
	uint32_t generation_end;
	uint32_t slot;
	uint16_t length;

	if (!topic_valid(topic) || buf == NULL || len == NULL) {
		return false;
	}

	store = &g_csyn_topics[topic];

	do {
		generation_start = (uint32_t)atomic_get(&store->generation);
		if (generation_start == 0U) {
			return false;
		}

		slot = generation_start & 1U;
		length = store->lengths[slot];
		if (length == 0U || length > buf_size) {
			return false;
		}

		memcpy(buf, store->slots[slot], length);
		generation_end = (uint32_t)atomic_get(&store->generation);
	} while (generation_start != generation_end);

	*len = length;
	if (generation != NULL) {
		*generation = generation_start;
	}

	return true;
}

uint32_t cubs2_csyn_topic_generation(enum cubs2_csyn_topic_id topic)
{
	if (!topic_valid(topic)) {
		return 0U;
	}

	return (uint32_t)atomic_get(&g_csyn_topics[topic].generation);
}

void cubs2_csyn_publish_flight_snapshot(const uint8_t *buf, size_t len)
{
	(void)cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_FLIGHT_SNAPSHOT, buf, len);
}

void cubs2_csyn_publish_motor_output(const uint8_t *buf, size_t len)
{
	(void)cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_MOTOR_OUTPUT, buf, len);
}

void cubs2_csyn_publish_control_output(const synapse_topic_RcChannels16_t *rc)
{
	(void)cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_CONTROL_OUTPUT, rc, sizeof(*rc));
}

void cubs2_csyn_publish_mocap_frame(const uint8_t *buf, size_t len)
{
	if (cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_MOCAP_FRAME, buf, len)) {
#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
		cubs2_host_deploy_io_put_mocap_payload(buf, len);
#endif
	}
}

void cubs2_csyn_publish_sim_input(const uint8_t *buf, size_t len)
{
	(void)cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_SIM_INPUT, buf, len);
}

void cubs2_csyn_publish_manual_control(const uint8_t *buf, size_t len)
{
	if (cubs2_csyn_topic_publish(CUBS2_CSYN_TOPIC_MANUAL_CONTROL, buf, len)) {
#if defined(CONFIG_CUBS2_HOST_DEPLOY_IO)
		synapse_topic_RcChannels16_t rc;
		bool valid;

		if (cubs2_topic_fb_unpack_manual_control(buf, len, &rc, &valid)) {
			cubs2_host_deploy_io_put_rc(&rc, valid);
		}
#endif
	}
}
