/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

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

#include "input_mapping.h"

#define MY_STACK_SIZE 3072
#define MY_PRIORITY   4
#define STATE_ANY     -1

LOG_MODULE_REGISTER(b3rb_fsm, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

static void transition(void *state, bool request, const char *request_name, int pre, uint8_t post,
		       char *buf, size_t n, int *request_seq, bool *request_rejected,
		       size_t n_guards, ...)
{
	uint8_t *state_int = (uint8_t *)state;

	// not requested
	if (!request) {
		return;
	}

	// null transition
	if (*state_int == post) {
		return;
	}

	// LOG_INF("%s pre: %d, post: %d, state: %d", request_name, pre, post, *state_int);
	// pre state required and not matched
	if (pre >= 0 && *state_int != pre) {
		return;
	}

	// new valid request
	(*request_seq)++;

	// check guards
	va_list ap;
	va_start(ap, n_guards);
	for (size_t i = 0; i < n_guards; i++) {
		int guard = va_arg(ap, int);
		const char *guard_txt = va_arg(ap, const char *);
		if (guard) {
			snprintf(buf, n, "deny %s: %s", request_name, guard_txt);
			LOG_WRN("%s", buf);
			*request_rejected = true;
			return;
		}
	}
	va_end(ap);
	snprintf(buf, n, "accept %s", request_name);
	LOG_INF("%s", buf);
	*state_int = post;
	*request_rejected = false;
	return;
}

struct status_input {
	struct input_request req;
	bool armed;
	bool safe;
	bool mode_unknown;
	bool mode_calibration;
	bool fuel_low;
	bool fuel_critical;

	bool update_battery_state;
	bool update_safety;
	bool update_input_sbus;
	bool update_input_ethernet;
	bool update_cmd_vel_ethernet;
	bool topic_source_ethernet;
	bool topic_source_input;
	bool input_timeout;
	bool input_status_loss;
	bool input_source_ethernet;
	bool input_source_radio_control;
	bool topic_timeout;
	bool topic_status_loss;

	// derived booleans
	bool update_input_from_ethernet;
	bool update_input_from_sbus;
	bool update_input;
	bool input_regained_now;
	bool input_lost_now;
	bool update_topic_from_ethernet;
	bool update_topic;
	bool topic_regained_now;
	bool topic_lost_now;
};

struct context {
	struct zros_node node;
	synapse_pb_Input input, input_sbus, input_ethernet;
	synapse_pb_BatteryState battery_state;
	synapse_pb_Safety safety;
	synapse_pb_Status status;
	synapse_pb_Twist cmd_vel, cmd_vel_ethernet;
	struct status_input status_input;
	struct zros_sub sub_input_sbus, sub_input_ethernet, sub_battery_state, sub_safety,
		sub_cmd_vel_ethernet;
	struct zros_pub pub_status, pub_input, pub_cmd_vel;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	int64_t now_ticks;
	int64_t input_last_ticks;
	const int64_t input_loss_ticks;
	int64_t topic_last_ticks;
	const int64_t topic_loss_ticks;
};

static struct context g_ctx = {
	.node = {},
	.input = synapse_pb_Input_init_default,
	.battery_state = synapse_pb_BatteryState_init_default,
	.safety = synapse_pb_Safety_init_default,
	.status =
		{
			.has_stamp = true,
			.stamp = synapse_pb_Timestamp_init_default,
			.arming = synapse_pb_Status_Arming_ARMING_DISARMED,
			.fuel = synapse_pb_Status_Fuel_FUEL_UNKNOWN,
			.fuel_percentage = 0,
			.flag = 0,
			.input_status = synapse_pb_Status_LinkStatus_STATUS_UNKNOWN,
			.input_source = synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET,
			.topic_source = synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT,
			.mode = synapse_pb_Status_Mode_MODE_ACTUATORS,
			.power = 0.0,
			.safety = synapse_pb_Status_Safety_SAFETY_UNKNOWN,
			.status_message = "",
		},
	.sub_battery_state = {},
	.sub_safety = {},
	.sub_input_sbus = {},
	.sub_input_ethernet = {},
	.sub_cmd_vel_ethernet = {},
	.pub_status = {},
	.pub_input = {},
	.pub_cmd_vel = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
	.now_ticks = 0,
	.input_last_ticks = 0,
	.topic_last_ticks = 0,
	.input_loss_ticks = 1.0 * CONFIG_SYS_CLOCK_TICKS_PER_SEC,
	.topic_loss_ticks = 1.0 * CONFIG_SYS_CLOCK_TICKS_PER_SEC,
};

static void b3rb_fsm_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "b3rb_fsm");
	zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state,
		      &ctx->battery_state, 10);
	zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
	zros_sub_init(&ctx->sub_input_sbus, &ctx->node, &topic_input_sbus, &ctx->input_sbus, 10);
	zros_sub_init(&ctx->sub_input_ethernet, &ctx->node, &topic_input_ethernet,
		      &ctx->input_ethernet, 10);
	zros_sub_init(&ctx->sub_cmd_vel_ethernet, &ctx->node, &topic_cmd_vel_ethernet,
		      &ctx->cmd_vel_ethernet, 10);
	zros_pub_init(&ctx->pub_status, &ctx->node, &topic_status, &ctx->status);
	zros_pub_init(&ctx->pub_input, &ctx->node, &topic_input, &ctx->input);
	zros_pub_init(&ctx->pub_cmd_vel, &ctx->node, &topic_cmd_vel, &ctx->cmd_vel);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void b3rb_fsm_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_battery_state);
	zros_sub_fini(&ctx->sub_safety);
	zros_sub_fini(&ctx->sub_input_sbus);
	zros_sub_fini(&ctx->sub_input_ethernet);
	zros_sub_fini(&ctx->sub_cmd_vel_ethernet);
	zros_pub_fini(&ctx->pub_status);
	zros_pub_fini(&ctx->pub_input);
	zros_pub_fini(&ctx->pub_cmd_vel);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void fsm_compute_input(struct status_input *input, struct context *ctx)
{
	input_request_compute(&input->req, &ctx->input);

	// precompute logical states
	input->armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;

	input->mode_unknown = ctx->status.mode == synapse_pb_Status_Mode_MODE_UNKNOWN;
	input->mode_calibration = ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
	input->safe = ctx->safety.status == synapse_pb_Safety_Status_SAFETY_SAFE ||
		      ctx->safety.status == synapse_pb_Safety_Status_SAFETY_UNKNOWN;
#else
	input->safe = false;
#endif

#ifdef CONFIG_CEREBRI_SENSE_POWER
	input->fuel_low =
		ctx->battery_state.voltage < CONFIG_CEREBRI_B3RB_BATTERY_LOW_MILLIVOLT / 1000.0;
	input->fuel_critical =
		ctx->battery_state.voltage < CONFIG_CEREBRI_B3RB_BATTERY_MIN_MILLIVOLT / 1000.0;
#else
	input->fuel_low = false;
	input->fuel_critical = false;
#endif

	// current ticks
	ctx->now_ticks = k_uptime_ticks();

	// booleans from input
	input->update_battery_state = zros_sub_update_available(&ctx->sub_battery_state);
	input->update_safety = zros_sub_update_available(&ctx->sub_safety);
	input->update_input_sbus = zros_sub_update_available(&ctx->sub_input_sbus);
	input->update_input_ethernet = zros_sub_update_available(&ctx->sub_input_ethernet);
	input->update_cmd_vel_ethernet = zros_sub_update_available(&ctx->sub_cmd_vel_ethernet);
	input->topic_source_ethernet =
		ctx->status.topic_source == synapse_pb_Status_TopicSource_TOPIC_SOURCE_ETHERNET;
	input->topic_source_input =
		ctx->status.topic_source == synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT;
	input->input_timeout = (ctx->now_ticks - ctx->input_last_ticks) > ctx->input_loss_ticks;
	input->input_status_loss =
		ctx->status.input_status == synapse_pb_Status_LinkStatus_STATUS_LOSS;
	input->input_source_ethernet =
		ctx->status.input_source == synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET;
	input->input_source_radio_control =
		ctx->status.input_source ==
		synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL;
	input->topic_timeout = (ctx->now_ticks - ctx->topic_last_ticks) > ctx->topic_loss_ticks;
	input->topic_status_loss =
		ctx->status.topic_status == synapse_pb_Status_LinkStatus_STATUS_LOSS;

	// derived booleans
	input->update_input_from_ethernet =
		input->update_input_ethernet && input->input_source_ethernet;
	input->update_input_from_sbus =
		input->update_input_sbus && input->input_source_radio_control;
	input->update_input = input->update_input_from_sbus || input->update_input_from_ethernet;
	input->input_regained_now = input->update_input && input->input_status_loss;
	input->input_lost_now = !input->input_status_loss && input->input_timeout;
	input->update_topic_from_ethernet =
		input->update_cmd_vel_ethernet && input->topic_source_ethernet;
	input->update_topic = input->update_cmd_vel_ethernet;
	input->topic_regained_now = input->update_topic && input->topic_status_loss;
	input->topic_lost_now = !input->topic_status_loss && input->topic_timeout;
}

static void fsm_update(synapse_pb_Status *status, const struct status_input *input)
{
	// arm transition
	transition(&status->arming,                                        // state
		   input->req.arm,                                         // request
		   "request arm",                                          // label
		   synapse_pb_Status_Arming_ARMING_DISARMED,               // pre
		   synapse_pb_Status_Arming_ARMING_ARMED,                  // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   // guards
		   5, input->mode_unknown, "mode not set", input->mode_calibration,
		   "mode calibration", input->safe, "safety on", input->fuel_critical,
		   "fuel_critical", input->fuel_low, "fuel_low");

	// disarm transitions
	transition(&status->arming,                                        // state
		   input->req.disarm,                                      // request
		   "request disarm",                                       // label
		   synapse_pb_Status_Arming_ARMING_ARMED,                  // pre
		   synapse_pb_Status_Arming_ARMING_DISARMED,               // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	transition(&status->arming,                                        // state
		   input->fuel_critical,                                   // request
		   "disarm fuel critical",                                 // label
		   synapse_pb_Status_Arming_ARMING_ARMED,                  // pre
		   synapse_pb_Status_Arming_ARMING_DISARMED,               // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	transition(&status->arming,                                        // state
		   input->safe,                                            // request
		   "disarm safety engaged",                                // label
		   synapse_pb_Status_Arming_ARMING_ARMED,                  // pre
		   synapse_pb_Status_Arming_ARMING_DISARMED,               // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	// mode transitions
	transition(&status->mode,                                          // state
		   input->req.mode_actuators,                              // request
		   "request mode actuators",                               // label
		   STATE_ANY,                                              // pre
		   synapse_pb_Status_Mode_MODE_ACTUATORS,                  // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	transition(&status->mode,                                          // state
		   input->req.mode_velocity,                               // request
		   "request mode velocity",                                // label
		   STATE_ANY,                                              // pre
		   synapse_pb_Status_Mode_MODE_VELOCITY,                   // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	transition(&status->mode,                                          // state
		   input->req.mode_bezier,                                 // request
		   "request mode bezier",                                  // label
		   STATE_ANY,                                              // pre
		   synapse_pb_Status_Mode_MODE_BEZIER,                     // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   0);                                                     // guards

	transition(&status->mode,                                          // state
		   input->req.mode_calibration,                            // request
		   "request mode calibration",                             // label
		   STATE_ANY,                                              // pre
		   synapse_pb_Status_Mode_MODE_CALIBRATION,                // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   // guards
		   1, input->armed, "disarm required");

	// topic source transitions
	transition(&status->topic_source,                                  // state
		   input->req.topic_source_input,                          // request
		   "request topic source input",                           // label
		   synapse_pb_Status_TopicSource_TOPIC_SOURCE_ETHERNET,    // pre
		   synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT,       // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   // guards
		   1, input->input_status_loss, "no input data");

	transition(&status->topic_source,                                  // state
		   input->req.topic_source_ethernet,                       // request
		   "request topic source ethernet",                        // label
		   synapse_pb_Status_TopicSource_TOPIC_SOURCE_INPUT,       // pre
		   synapse_pb_Status_TopicSource_TOPIC_SOURCE_ETHERNET,    // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   // guards
		   1, input->topic_status_loss, "no topic data");

	// input source transitions
	transition(&status->input_source,                                    // state
		   input->req.input_source_radio_control,                    // request
		   "request input source radio control",                     // label
		   STATE_ANY,                                                // pre
		   synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL, // post
		   status->status_message, sizeof(status->status_message),   // status
		   &status->request_seq, &status->request_rejected,          // request
		   // guards
		   1, input->input_status_loss, "no RC input data");

	transition(&status->input_source,                                  // state
		   input->req.input_source_ethernet,                       // request
		   "request input source ethernet",                        // label
		   STATE_ANY,                                              // pre
		   synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET,    // post
		   status->status_message, sizeof(status->status_message), // status
		   &status->request_seq, &status->request_rejected,        // request
		   // guards
		   1, input->input_status_loss, "no enet input data");

	// set timestamp
	stamp_msg(&status->stamp, k_uptime_ticks());
}

static void status_add_extra_info(synapse_pb_Status *status, struct status_input *input,
				  const struct context *ctx)
{
	if (input->req.lights_on) {
		status->flag |= synapse_pb_Status_Flag_FLAG_LIGHTING;
	} else {
		status->flag &= ~synapse_pb_Status_Flag_FLAG_LIGHTING;
	}
	if (input->fuel_critical) {
		status->fuel = synapse_pb_Status_Fuel_FUEL_CRITICAL;
	} else if (input->fuel_low) {
		status->fuel = synapse_pb_Status_Fuel_FUEL_LOW;
	} else {
		status->fuel = synapse_pb_Status_Fuel_FUEL_NOMINAL;
	}
	double bat_max = CONFIG_CEREBRI_B3RB_BATTERY_MAX_MILLIVOLT / 1000.0;
	double bat_min = CONFIG_CEREBRI_B3RB_BATTERY_MIN_MILLIVOLT / 1000.0;
	status->fuel_percentage =
		100 * (ctx->battery_state.voltage - bat_min) / (bat_max - bat_min);
	status->power = ctx->battery_state.voltage * ctx->battery_state.current;
#ifdef CONFIG_CEREBRI_SENSE_SAFETY
	if (ctx->safety.status == synapse_pb_Safety_Status_SAFETY_SAFE) {
		status->safety = synapse_pb_Status_Safety_SAFETY_SAFE;
	} else if (ctx->safety.status == synapse_pb_Safety_Status_SAFETY_UNSAFE) {
		status->safety = synapse_pb_Status_Safety_SAFETY_UNSAFE;
	} else if (ctx->safety.status == synapse_pb_Safety_Status_SAFETY_UNKNOWN) {
		status->safety = synapse_pb_Status_Safety_SAFETY_UNKNOWN;
	}
#else
	status->safety = synapse_pb_Status_Safety_SAFETY_UNSAFE;
#endif
}

static void b3rb_fsm_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	b3rb_fsm_init(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_input_sbus),
		*zros_sub_get_event(&ctx->sub_input_ethernet),
		*zros_sub_get_event(&ctx->sub_battery_state),
	};

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

		// wait for input event, publish at 1 Hz regardless
		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG("fsm polling timeout");
		}

		// perform finite state machine processing
		fsm_compute_input(&ctx->status_input, ctx);
		fsm_update(&ctx->status, &ctx->status_input);
		status_add_extra_info(&ctx->status, &ctx->status_input, ctx);
		zros_pub_update(&ctx->pub_status);

		struct status_input *in = &ctx->status_input;

		// handle update based on computed booleans
		if (in->update_battery_state) {
			zros_sub_update(&ctx->sub_battery_state);
		}

		if (in->update_safety) {
			zros_sub_update(&ctx->sub_safety);
		}

		if (in->update_input_sbus) {
			zros_sub_update(&ctx->sub_input_sbus);
		}

		if (in->update_input_ethernet) {
			zros_sub_update(&ctx->sub_input_ethernet);
		}

		if (in->update_cmd_vel_ethernet) {
			zros_sub_update(&ctx->sub_cmd_vel_ethernet);
		}

		// update input from correct source
		if (in->update_input_from_ethernet) {
			ctx->input = ctx->input_ethernet;
		} else if (in->update_input_from_sbus) {
			ctx->input = ctx->input_sbus;
		}

		// update ticks
		if (in->update_input) {
			ctx->input_last_ticks = ctx->now_ticks;
		}

		// check for input regained
		if (in->input_regained_now) {
			LOG_INF("input regained");
			ctx->status.input_status = synapse_pb_Status_LinkStatus_STATUS_NOMINAL;
		}

		// check for input loss
		if (in->input_lost_now) {
			LOG_INF("input loss");
			ctx->status.input_status = synapse_pb_Status_LinkStatus_STATUS_LOSS;
		}

		// if signal loss, try to reconfigure to another link
		if (in->input_status_loss) {
			if (in->update_input_sbus) {
				ctx->status.input_source =
					synapse_pb_Status_InputSource_INPUT_SOURCE_RADIO_CONTROL;
				LOG_INF("reconfiguring link to use radio control");
			} else if (in->update_input_ethernet) {
				ctx->status.input_source =
					synapse_pb_Status_InputSource_INPUT_SOURCE_ETHERNET;
				LOG_INF("reconfiguring link to use ethernet");
			}
		}

		// update topic from correct source
		if (in->update_topic_from_ethernet) {
			ctx->cmd_vel = ctx->cmd_vel_ethernet;
			zros_pub_update(&ctx->pub_cmd_vel);
		}

		// update ticks
		if (in->update_topic) {
			ctx->topic_last_ticks = ctx->now_ticks;
		}

		// check for topic regained
		if (in->topic_regained_now) {
			LOG_INF("topic regained");
			ctx->status.topic_status = synapse_pb_Status_LinkStatus_STATUS_NOMINAL;
		}

		// check for topic loss
		if (in->topic_lost_now) {
			LOG_INF("topic loss");
			ctx->status.topic_status = synapse_pb_Status_LinkStatus_STATUS_LOSS;
		}

		// publish control topics
		if (in->update_topic && in->topic_source_ethernet) {
			ctx->cmd_vel = ctx->cmd_vel_ethernet;
			zros_pub_update(&ctx->pub_cmd_vel);
		} else if (in->update_input && in->topic_source_input) {
			zros_pub_update(&ctx->pub_input);
		}
	}

	b3rb_fsm_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      b3rb_fsm_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "b3rb_fsm");
	k_thread_start(tid);
	return 0;
}

static int b3rb_fsm_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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

SHELL_SUBCMD_DICT_SET_CREATE(sub_b3rb_fsm, b3rb_fsm_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(b3rb_fsm, &sub_b3rb_fsm, "b3rb fsm commands", NULL);

static int b3rb_fsm_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(b3rb_fsm_sys_init, APPLICATION, 1);

// vi: ts=4 sw=4 et
