/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rc_input.h"

#include "topic_bus.h"

#include <zephyr/drivers/input/input_crsf.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#define RC_NODE                DT_ALIAS(rc)
#define RC_CHANNEL_COUNT       16
#define THROTTLE_CHANNEL_INDEX 2
#define RC_US_CENTER           1500
#define RC_US_MIN              1000

static synapse_topic_RcChannels16_t g_rc_staging;
static synapse_topic_RcChannels16_t g_rc_latest;
static bool g_rc_staging_valid;
static bool g_rc_latest_valid;
static int64_t g_rc_latest_stamp_ms;
static atomic_t g_rc_seq;
static atomic_t g_rc_link_quality;
static struct zros_node g_rdd2_rc_node;
static struct zros_pub g_rdd2_rc_pub;
static synapse_topic_RcChannels16_t g_rdd2_rc_msg;
static bool g_rdd2_rc_pub_ready;

static void rc_channels_set_defaults(synapse_topic_RcChannels16_t *rc, bool *valid,
				     int64_t *stamp_ms)
{
	int32_t *channels = rdd2_topic_rc_channels_data(rc);

	for (size_t i = 0; i < RC_CHANNEL_COUNT; i++) {
		channels[i] = RC_US_CENTER;
	}

	channels[THROTTLE_CHANNEL_INDEX] = RC_US_MIN;
	if (valid != NULL) {
		*valid = false;
	}
	if (stamp_ms != NULL) {
		*stamp_ms = 0;
	}
}

void rdd2_rc_input_init(void)
{
	int rc;

	rc_channels_set_defaults(&g_rc_staging, &g_rc_staging_valid, NULL);
	rc_channels_set_defaults(&g_rc_latest, &g_rc_latest_valid, &g_rc_latest_stamp_ms);
	atomic_set(&g_rc_seq, 0);
	atomic_set(&g_rc_link_quality, 0);
	zros_node_init(&g_rdd2_rc_node, "rdd2_rc_input");
	rc = zros_pub_init(&g_rdd2_rc_pub, &g_rdd2_rc_node, &topic_rc, &g_rdd2_rc_msg);
	g_rdd2_rc_pub_ready = (rc == 0);
}

void rdd2_rc_input_latest_get(synapse_topic_RcChannels16_t *rc, int64_t *stamp_ms, bool *valid)
{
	atomic_val_t seq_start;
	atomic_val_t seq_end = 0;

	if (rc == NULL || stamp_ms == NULL || valid == NULL) {
		return;
	}

	do {
		seq_start = atomic_get(&g_rc_seq);
		if ((seq_start & 1) != 0) {
			continue;
		}

		*rc = g_rc_latest;
		*stamp_ms = g_rc_latest_stamp_ms;
		*valid = g_rc_latest_valid;
		seq_end = atomic_get(&g_rc_seq);
	} while (seq_start != seq_end);
}

uint8_t rdd2_rc_input_link_quality_get(const struct device *dev)
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
	int32_t *staging_channels = rdd2_topic_rc_channels_data(&g_rc_staging);

	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_ABS && evt->code >= 1 && evt->code <= RC_CHANNEL_COUNT) {
		staging_channels[evt->code - 1] = evt->value;
		g_rc_staging_valid = true;
	} else if (evt->type == INPUT_EV_MSC && evt->code == RDD2_RC_INPUT_EVENT_LINK_QUALITY) {
		atomic_set(&g_rc_link_quality, evt->value);
	} else if (evt->type == INPUT_EV_MSC && evt->code == RDD2_RC_INPUT_EVENT_VALID) {
		g_rc_staging_valid = evt->value != 0;
	}

	if (evt->sync) {
		atomic_inc(&g_rc_seq);
		g_rc_latest = g_rc_staging;
		g_rc_latest_valid = g_rc_staging_valid;
		g_rc_latest_stamp_ms = k_uptime_get();
		atomic_inc(&g_rc_seq);
		if (g_rdd2_rc_pub_ready) {
			g_rdd2_rc_msg = g_rc_latest;
			(void)zros_pub_update(&g_rdd2_rc_pub);
		}
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(RC_NODE), rc_input_cb, NULL);
