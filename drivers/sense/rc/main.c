/*
 * Copyright CogniPilot Foundation 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#if DT_NODE_HAS_COMPAT(DT_ALIAS(rc), tbs_crsf)
#include "lib/core/common/casadi/common.h"
#include <cerebri/core/casadi.h>
#include <zephyr/drivers/input/input_crsf.h>
#include <zephyr/sys/byteorder.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "crsf_yaapu.h"
#define CRSF_TELEMETRY 1

enum crsf_telem {
	CRSF_ATTITUDE,
	CRSF_BATTERY,
	CRSF_FLIGHT_MODE,
	CRSF_GPS,
	CRSF_YAAPU_ATTITUDE,
	CRSF_YAAPU_GPS,
	CRSF_YAAPU_STATUS_BAT,
	CRSF_YAAPU_STATUSTEXT,
};

struct telem_schedule_entry {
	enum crsf_telem id;    // The ID used in the switch case
	float weight;          // Higher weight = sent more frequently
	int64_t min_period_ms; // Minimum time between sends

	// Internal Scheduler State
	float virtual_finish_time;
	int64_t last_sent_ms;
};

// TODO tune this, maybe make a base crsf telem and yaapu telem schedule
static struct telem_schedule_entry telem_schedule[] = {
	// ID                           Weight   Min Period (ms)  Notes
	//{CRSF_ATTITUDE,                 10.0f,   100,             /* 10 Hz */},
	{CRSF_GPS, 4.0f, 250, /* 4 Hz */},
	{CRSF_BATTERY, 1.0f, 1000, /* 1 Hz */},
	{CRSF_FLIGHT_MODE, 1.0f, 1000, /* 1 Hz */},
	{CRSF_YAAPU_ATTITUDE, 10.0f, 100, /* 10 Hz (Yaapu) */},
	{CRSF_YAAPU_STATUSTEXT, 10.0f, 1000, /* 1 Hz (Yaapu) */},
	{CRSF_YAAPU_STATUS_BAT, 2.0f, 500, /* 2 Hz (Yaapu) */},
};

static const int telem_schedule_count = sizeof(telem_schedule) / sizeof(telem_schedule[0]);

#endif

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   2

LOG_MODULE_REGISTER(sense_rc, CONFIG_CEREBRI_SENSE_RC_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	struct zros_node node;
	struct zros_pub pub_input;
	synapse_pb_Input input;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	int last_event;
#ifdef CRSF_TELEMETRY
	enum crsf_telem telem_state;
	synapse_pb_BatteryState battery_state;
	synapse_pb_Status status;
	synapse_pb_Odometry odometry_estimator;
	struct zros_sub sub_battery_state, sub_status, sub_odometry_estimator;
	bool yaapu_startup;
	uint32_t yaapu_status_seqno;
#endif
};

static struct context g_ctx = {
	.node = {},
	.pub_input = {},
	.input = {.has_timestamp = true, .channel_count = 16, .channel = {}},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.last_event = 0,
#ifdef CRSF_TELEMETRY
	.telem_state = CRSF_GPS,
	.battery_state = synapse_pb_BatteryState_init_default,
	.status = synapse_pb_Status_init_default,
	.odometry_estimator = synapse_pb_Odometry_init_default,
	.sub_battery_state = {},
	.sub_status = {},
	.sub_odometry_estimator = {},
	.yaapu_startup = 0,
	.yaapu_status_seqno = 0,
#endif
};

static void sense_rc_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "sense_rc");
	zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input_rc, &ctx->input);
	ctx->last_event = 0;
#ifdef CRSF_TELEMETRY
	zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state,
		      &ctx->battery_state, 10);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_sub_init(&ctx->sub_odometry_estimator, &ctx->node, &topic_odometry_estimator,
		      &ctx->odometry_estimator, 10);
#endif
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void sense_rc_fini(struct context *ctx)
{
#ifdef CRSF_TELEMETRY
	zros_sub_fini(&ctx->sub_battery_state);
	zros_sub_fini(&ctx->sub_status);
	zros_sub_fini(&ctx->sub_odometry_estimator);
#endif
	zros_pub_fini(&ctx->pub_input);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void sense_rc_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	sense_rc_init(ctx);

#ifdef CRSF_TELEMETRY
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(rc));

	// Setup Yaapu configuration
	{
		struct crsf_ap_custom_multi_packet payload;
		payload.sub_id = CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH;
		payload.size = 2;
		payload.packets[0].appid = CRSF_YAAPU_PARAMS_APPID;
		payload.packets[0].data.params.param_id = 1; // Frame Type
		payload.packets[0].data.params.param_value =
			(uint8_t)('c'); // Copter TODO be more flexible
		payload.packets[1].appid = CRSF_YAAPU_PARAMS_APPID;
		payload.packets[1].data.params.param_id = 4;      // Batt 1 Cap
		payload.packets[1].data.params.param_value = 100; // now we cheat to get percentage
		input_crsf_send_telemetry(dev, CRSF_TYPE_AP_CUSTOM_TELEM, (uint8_t *)&payload,
					  2 + payload.size * 6);
	}

	// Initialize scheduler state (optional, ensures clean start)
	int64_t now = k_uptime_get();
	for (int i = 0; i < telem_schedule_count; i++) {
		telem_schedule[i].last_sent_ms = now - telem_schedule[i].min_period_ms;
		telem_schedule[i].virtual_finish_time = 0.0f;
	}

	// Loop runs while the 'running' semaphore is NOT given (timeout ensures periodic run)
	while (k_sem_take(&ctx->running, K_MSEC(10)) < 0) {

		now = k_uptime_get();
		struct telem_schedule_entry *best_candidate = NULL;
		float min_vft = FLT_MAX;

		// WFQ SCHEDULER  Find the eligible item with the smallest Virtual Finish Time
		for (int i = 0; i < telem_schedule_count; i++) {
			struct telem_schedule_entry *entry = &telem_schedule[i];

			if ((now - entry->last_sent_ms) < entry->min_period_ms) {
				continue;
			}

			if (entry->virtual_finish_time < min_vft) {
				min_vft = entry->virtual_finish_time;
				best_candidate = entry;
			}
		}

		// If no candidate is ready (all rate-limited), wait for next loop cycle
		if (!best_candidate) {
			continue;
		}

		switch (best_candidate->id) {

		case CRSF_ATTITUDE:
			break;

		case CRSF_BATTERY: {
			zros_sub_update(&ctx->sub_battery_state);
			struct crsf_payload_battery bat_payload;
			bat_payload.voltage_dV =
				sys_cpu_to_be16((int16_t)(ctx->battery_state.voltage * 10));
			bat_payload.current_dA =
				sys_cpu_to_be16((int16_t)(ctx->battery_state.current * 10));
			bat_payload.remaining_pct = (int8_t)(ctx->status.fuel_percentage);
			input_crsf_send_telemetry(dev, CRSF_TYPE_BATTERY, (uint8_t *)&bat_payload,
						  sizeof(struct crsf_payload_battery));
		} break;

		case CRSF_FLIGHT_MODE: {
			zros_sub_update(&ctx->sub_status);
			input_crsf_send_telemetry(dev, CRSF_TYPE_FLIGHT_MODE,
						  (uint8_t *)mode_str(ctx->status.mode),
						  strlen(mode_str(ctx->status.mode)) + 1);
		} break;

		case CRSF_GPS:
			// TODO Add GPS logic here
			break;

		case CRSF_YAAPU_ATTITUDE: {
			zros_sub_update(&ctx->sub_odometry_estimator);
			struct crsf_ap_custom_multi_packet payload;
			payload.sub_id = CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH;
			payload.size = 2;

			double q[4] = {ctx->odometry_estimator.pose.orientation.w,
				       ctx->odometry_estimator.pose.orientation.x,
				       ctx->odometry_estimator.pose.orientation.y,
				       ctx->odometry_estimator.pose.orientation.z};
			double yaw, pitch, roll;
			double rad2deg = 180 / 3.14159;
			CASADI_FUNC_ARGS(quat_to_eulerB321)
			args[0] = q;
			res[0] = &yaw;
			res[1] = &pitch;
			res[2] = &roll;
			CASADI_FUNC_CALL(quat_to_eulerB321)

			payload.packets[0].appid = CRSF_YAAPU_ROLLPITCH_APPID;
			payload.packets[0].data.roll_pitch.roll_bits =
				((rad2deg * roll) / 0.2) + 900;
			payload.packets[0].data.roll_pitch.pitch_bits =
				((rad2deg * pitch) / 0.2) + 450;
			payload.packets[1].appid = CRSF_YAAPU_VELANDYAW_APPID;
			payload.packets[1].data.vel_yaw.yaw_bits = (rad2deg * yaw) / 0.2;

			input_crsf_send_telemetry(dev, CRSF_TYPE_AP_CUSTOM_TELEM,
						  (uint8_t *)&payload, 2 + payload.size * 6);
		} break;

		case CRSF_YAAPU_STATUSTEXT: {
			zros_sub_update(&ctx->sub_status);
			if (ctx->status.request_seq > ctx->yaapu_status_seqno) {
				ctx->yaapu_status_seqno = ctx->status.request_seq;
				struct crsf_ap_custom_status_text payload;
				payload.sub_id = CRSF_AP_CUSTOM_TELEM_STATUS_TEXT;
				payload.severity = CRSF_AP_CUSTOM_SEVERITY_INFO;
				snprintf(payload.text, sizeof(payload.text), "%.49s",
					 ctx->status.status_message);
				input_crsf_send_telemetry(dev, CRSF_TYPE_AP_CUSTOM_TELEM,
							  (uint8_t *)&payload,
							  strlen(payload.text) + 3);
			}
		} break;

		case CRSF_YAAPU_STATUS_BAT: {
			zros_sub_update(&ctx->sub_status);
			zros_sub_update(&ctx->sub_battery_state);
			struct crsf_ap_custom_multi_packet payload;
			payload.sub_id = CRSF_AP_CUSTOM_TELEM_MULTI_PACKET_PASSTHROUGH;
			payload.size = 2;
			payload.packets[0].appid = CRSF_YAAPU_AP_STATUS_APPID;
			payload.packets[0].data.raw = 0x0;
			payload.packets[0].data.ap_status.armed =
				ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
			// TODO other status
			payload.packets[1].appid = CRSF_YAAPU_BATTERY_APPID;
			payload.packets[1].data.raw = 0x0;
			payload.packets[1].data.battery.voltage = (ctx->battery_state.voltage * 10);
			if (ctx->battery_state.current <= 12.7) {
				payload.packets[1].data.battery.current_exp = 0;
				payload.packets[1].data.battery.current_mantissa =
					lroundl(ctx->battery_state.current * 10);
			} else if (ctx->battery_state.current <= 127) {
				payload.packets[1].data.battery.current_exp = 1;
				payload.packets[1].data.battery.current_mantissa =
					lroundl(ctx->battery_state.current);
			} else {
				payload.packets[1].data.battery.current_exp = 1;
				payload.packets[1].data.battery.current_mantissa = 127; // Max value
			}
			payload.packets[1].data.battery.mah =
				100 - ctx->status.fuel_percentage; // Cheat to get percentage
			input_crsf_send_telemetry(dev, CRSF_TYPE_AP_CUSTOM_TELEM,
						  (uint8_t *)&payload, 2 + payload.size * 6);
		} break;

		default:
			break;
		}

		best_candidate->last_sent_ms = now;
		best_candidate->virtual_finish_time += (1.0f / best_candidate->weight);
	}

#else
	// wait for stop request
	while (k_sem_take(&ctx->running, K_MSEC(1000)) < 0)
		;
#endif

	sense_rc_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      sense_rc_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "sense_rc");
	k_thread_start(tid);
	return 0;
}

static void input_cb(struct input_event *evt, void *userdata)
{
	struct context *ctx = userdata;

	// check if still running
	if (k_sem_count_get(&ctx->running) != 0) {
		return;
	}

	float x0 = 1024;
	float scale = 784;

	if (evt->code > 0 && evt->code <= ctx->input.channel_count) {
		ctx->input.channel[evt->code - 1] = (evt->value - x0) / scale;
	} else {
		LOG_DBG("unhandled event: %d %d %d %d", evt->code, evt->sync, evt->type,
			evt->value);
	}

	if (evt->sync == true) {
		stamp_msg(&ctx->input.timestamp, k_uptime_ticks());
		zros_pub_update(&ctx->pub_input);
	}
	ctx->last_event = evt->code;
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_ALIAS(rc)), input_cb, &g_ctx);

static int sense_rc_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;

	if (strcmp(argv[0], "start") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			shell_print(sh, "already running");
		} else {
			start(ctx);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (k_sem_count_get(&g_ctx.running) == 0) {
			k_sem_give(&g_ctx.running);
		} else {
			shell_print(sh, "not running");
		}
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)k_sem_count_get(&g_ctx.running) == 0);
	}
	return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(sub_sense_rc, sense_rc_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(sense_rc, &sub_sense_rc, "sense rc args", NULL);

static int sense_rc_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(sense_rc_sys_init, APPLICATION, 2);

/* vi: ts=4 sw=4 et */
