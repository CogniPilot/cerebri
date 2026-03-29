/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rc_input.h"

#include "topic_shell.h"

#include <zephyr/drivers/input/input_crsf.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#define RC_NODE                DT_ALIAS(rc)
#define RC_CHANNEL_COUNT       16
#define THROTTLE_CHANNEL_INDEX 2
#define RC_US_CENTER           1500
#define RC_US_MIN              1000

static struct rc_frame g_rc_staging = {
	.us = { [0 ... RC_CHANNEL_COUNT - 1] = RC_US_CENTER },
};
static struct rc_frame g_rc_latest = {
	.us = { [0 ... RC_CHANNEL_COUNT - 1] = RC_US_CENTER },
};
static atomic_t g_rc_seq;
static atomic_t g_rc_link_quality;

static void rc_frame_set_defaults(struct rc_frame *frame)
{
	for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
		frame->us[i] = RC_US_CENTER;
	}

	frame->us[THROTTLE_CHANNEL_INDEX] = RC_US_MIN;
	frame->valid = false;
	frame->stamp_ms = 0;
}

void cerebri2_rc_input_init(void)
{
	rc_frame_set_defaults(&g_rc_staging);
	rc_frame_set_defaults(&g_rc_latest);
	atomic_set(&g_rc_seq, 0);
	atomic_set(&g_rc_link_quality, 0);
}

struct rc_frame cerebri2_rc_input_snapshot(void)
{
	struct rc_frame frame;
	atomic_val_t seq_start;
	atomic_val_t seq_end = 0;

	do {
		seq_start = atomic_get(&g_rc_seq);
		if ((seq_start & 1) != 0) {
			continue;
		}

		frame = g_rc_latest;
		seq_end = atomic_get(&g_rc_seq);
	} while (seq_start != seq_end);

	return frame;
}

uint8_t cerebri2_rc_input_link_quality_get(const struct device *dev)
{
#if DT_NODE_HAS_COMPAT(RC_NODE, tbs_crsf)
	return input_crsf_get_link_stats(dev).uplink_link_quality;
#else
	ARG_UNUSED(dev);
	return (uint8_t)atomic_get(&g_rc_link_quality);
#endif
}

static void rc_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_ABS && evt->code >= 1 && evt->code <= RC_CHANNEL_COUNT) {
		g_rc_staging.us[evt->code - 1] = evt->value;
		g_rc_staging.valid = true;
	} else if (evt->type == INPUT_EV_MSC && evt->code == CEREBRI2_RC_INPUT_EVENT_LINK_QUALITY) {
		atomic_set(&g_rc_link_quality, evt->value);
	} else if (evt->type == INPUT_EV_MSC && evt->code == CEREBRI2_RC_INPUT_EVENT_VALID) {
		g_rc_staging.valid = evt->value != 0;
	}

	if (evt->sync) {
		atomic_inc(&g_rc_seq);
		g_rc_latest = g_rc_staging;
		g_rc_latest.stamp_ms = k_uptime_get();
		atomic_inc(&g_rc_seq);
		cerebri2_topic_rc_published();
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(RC_NODE), rc_input_cb, NULL);
