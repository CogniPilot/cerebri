#ifndef CUBS2_CSYN_H_
#define CUBS2_CSYN_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include "topic_flatbuffer.h"

#define CUBS2_CSYN_TOPIC_MAX_SIZE CUBS2_TOPIC_FB_MOCAP_FRAME_MAX_SIZE

enum cubs2_csyn_topic_id {
	CUBS2_CSYN_TOPIC_FLIGHT_SNAPSHOT = 0,
	CUBS2_CSYN_TOPIC_MOTOR_OUTPUT,
	CUBS2_CSYN_TOPIC_CONTROL_OUTPUT,
	CUBS2_CSYN_TOPIC_MOCAP_FRAME,
	CUBS2_CSYN_TOPIC_SIM_INPUT,
	CUBS2_CSYN_TOPIC_MANUAL_CONTROL,
	CUBS2_CSYN_TOPIC_COUNT,
	CUBS2_CSYN_TOPIC_INVALID = -1,
};

enum cubs2_csyn_topic_kind {
	CUBS2_CSYN_KIND_FLATBUFFER = 0,
	CUBS2_CSYN_KIND_STRUCT,
	CUBS2_CSYN_KIND_OPAQUE,
};

struct cubs2_csyn_topic_info {
	const char *name;
	const char *keyexpr;
	const char *type_name;
	enum cubs2_csyn_topic_kind kind;
	size_t max_size;
};

#if defined(CONFIG_CUBS2_CSYN)
enum cubs2_csyn_topic_id cubs2_csyn_topic_parse(const char *name);
const struct cubs2_csyn_topic_info *cubs2_csyn_topic_info(enum cubs2_csyn_topic_id topic);
size_t cubs2_csyn_topic_count(void);
bool cubs2_csyn_topic_publish(enum cubs2_csyn_topic_id topic, const void *buf, size_t len);
bool cubs2_csyn_topic_copy(enum cubs2_csyn_topic_id topic, void *buf, size_t buf_size,
			   size_t *len, uint32_t *generation);
uint32_t cubs2_csyn_topic_generation(enum cubs2_csyn_topic_id topic);
void cubs2_csyn_publish_flight_snapshot(const uint8_t *buf, size_t len);
void cubs2_csyn_publish_motor_output(const uint8_t *buf, size_t len);
void cubs2_csyn_publish_control_output(const synapse_topic_RcChannels16_t *rc);
void cubs2_csyn_publish_mocap_frame(const uint8_t *buf, size_t len);
void cubs2_csyn_publish_sim_input(const uint8_t *buf, size_t len);
void cubs2_csyn_publish_manual_control(const uint8_t *buf, size_t len);
#else
static inline enum cubs2_csyn_topic_id cubs2_csyn_topic_parse(const char *name)
{
	ARG_UNUSED(name);
	return CUBS2_CSYN_TOPIC_INVALID;
}

static inline const struct cubs2_csyn_topic_info *cubs2_csyn_topic_info(
	enum cubs2_csyn_topic_id topic)
{
	ARG_UNUSED(topic);
	return NULL;
}

static inline size_t cubs2_csyn_topic_count(void)
{
	return 0U;
}

static inline bool cubs2_csyn_topic_publish(enum cubs2_csyn_topic_id topic, const void *buf,
					    size_t len)
{
	ARG_UNUSED(topic);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return false;
}

static inline bool cubs2_csyn_topic_copy(enum cubs2_csyn_topic_id topic, void *buf,
					 size_t buf_size, size_t *len, uint32_t *generation)
{
	ARG_UNUSED(topic);
	ARG_UNUSED(buf);
	ARG_UNUSED(buf_size);
	ARG_UNUSED(len);
	ARG_UNUSED(generation);
	return false;
}

static inline uint32_t cubs2_csyn_topic_generation(enum cubs2_csyn_topic_id topic)
{
	ARG_UNUSED(topic);
	return 0U;
}

static inline void cubs2_csyn_publish_flight_snapshot(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}

static inline void cubs2_csyn_publish_motor_output(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}

static inline void cubs2_csyn_publish_control_output(const synapse_topic_RcChannels16_t *rc)
{
	ARG_UNUSED(rc);
}

static inline void cubs2_csyn_publish_mocap_frame(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}

static inline void cubs2_csyn_publish_sim_input(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}

static inline void cubs2_csyn_publish_manual_control(const uint8_t *buf, size_t len)
{
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
}
#endif

#endif
