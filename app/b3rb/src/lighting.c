/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

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

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   4

LOG_MODULE_REGISTER(b3rb_lighting, CONFIG_CEREBRI_B3RB_LOG_LEVEL);

static K_THREAD_STACK_DEFINE(g_my_stack_area, MY_STACK_SIZE);

struct context {
	// node
	struct zros_node node;
	// data
	synapse_pb_BatteryState battery_state;
	synapse_pb_Safety safety;
	synapse_pb_Status status;
	synapse_pb_LEDArray led_array;
	// subscriptions
	struct zros_sub sub_battery_state, sub_safety, sub_status;
	// publications
	struct zros_pub pub_led_array;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
};

static struct context g_ctx = {
	.node = {},
	.battery_state = synapse_pb_BatteryState_init_default,
	.safety = synapse_pb_Safety_init_default,
	.status = synapse_pb_Status_init_default,
	.led_array = synapse_pb_LEDArray_init_default,
	.sub_safety = {},
	.sub_status = {},
	.pub_led_array = {},
	.running = Z_SEM_INITIALIZER(g_ctx.running, 1, 1),
	.stack_size = MY_STACK_SIZE,
	.stack_area = g_my_stack_area,
	.thread_data = {},
};

static void b3rb_lighting_init(struct context *ctx)
{
	zros_node_init(&ctx->node, "b3rb_lighting");
	zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state,
		      &ctx->battery_state, 10);
	zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 10);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_pub_init(&ctx->pub_led_array, &ctx->node, &topic_led_array, &ctx->led_array);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
}

static void b3rb_lighting_fini(struct context *ctx)
{
	zros_sub_fini(&ctx->sub_battery_state);
	zros_sub_fini(&ctx->sub_safety);
	zros_sub_fini(&ctx->sub_status);
	zros_pub_fini(&ctx->pub_led_array);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
}

static void set_led(const int index, const double *color, const double brightness,
		    synapse_pb_LEDArray_LED *led)
{
	led->index = index;
	led->r = brightness * color[0];
	led->g = brightness * color[1];
	led->b = brightness * color[2];
}

static void b3rb_lighting_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	b3rb_lighting_init(ctx);

	while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {

		// wait 33 ms
		k_msleep(33);

		// update subscriptions
		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}

		if (zros_sub_update_available(&ctx->sub_safety)) {
			zros_sub_update(&ctx->sub_safety);
		}

		if (zros_sub_update_available(&ctx->sub_battery_state)) {
			zros_sub_update(&ctx->sub_battery_state);
		}

		// timing
		double t = k_uptime_ticks() / ((double)CONFIG_SYS_CLOCK_TICKS_PER_SEC);
		const double led_pulse_freq = 0.25;
		const double brightness_min = 4;
		const double brightness_max = 30;
		const double brightness_amplitude = (brightness_max - brightness_min) / 2;
		const double brightness_mean = (brightness_max + brightness_min) / 2;

		int brightness = brightness_mean +
				 brightness_amplitude * sin(2 * 3.14159 * led_pulse_freq * t);
		int led_msg_index = 0;

		const int mode_leds[] = {2, 3};
		const double color_auto[] = {1, 0, 0};
		const double color_manual[] = {0, 1, 0};
		const double color_cmd_vel[] = {0, 0, 1};
		const double color_unknown[] = {0.33, 0.33, 0.33};

		const int arm_leds[] = {1, 4};
		const double color_armed[] = {1, 0, 0};
		const double color_disarmed[] = {0, 1, 0};

		const int safety_leds[] = {0, 5};
		const double color_unsafe[] = {1, 0, 0};
		const double color_safe[] = {0, 1, 0};
		const double color_battery_critical[] = {1, 0.65, 0};
		const double color_calibration[] = {1, 1, 0};

		const int headlight_leds[] = {6, 7, 8, 9, 10, 11};
		const double color_white[] = {1, 1, 1};

		bool battery_critical = ctx->battery_state.voltage <
					CONFIG_CEREBRI_B3RB_BATTERY_MIN_MILLIVOLT / 1000.0;

		// mode leds
		for (size_t i = 0; i < ARRAY_SIZE(mode_leds); i++) {
			const double *color = NULL;
			if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ACTUATORS) {
				color = color_manual;
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_VELOCITY) {
				color = color_cmd_vel;
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_BEZIER) {
				color = color_auto;
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION) {
				color = color_calibration;
			} else {
				color = color_unknown;
			}
			set_led(mode_leds[i], color, brightness,
				&ctx->led_array.led[led_msg_index]);
			led_msg_index++;
		}

		// arm leds
		for (size_t i = 0; i < ARRAY_SIZE(arm_leds); i++) {
			const double *color = NULL;
			if (battery_critical) {
				color = color_battery_critical;
			} else {
				if (ctx->status.arming ==
				    synapse_pb_Status_Arming_ARMING_DISARMED) {
					color = color_disarmed;
				} else if (ctx->status.arming ==
					   synapse_pb_Status_Arming_ARMING_ARMED) {
					color = color_armed;
				} else {
					color = color_unknown;
				}
			}
			set_led(arm_leds[i], color, brightness, &ctx->led_array.led[led_msg_index]);
			led_msg_index++;
		}

		// safety leds
		for (size_t i = 0; i < ARRAY_SIZE(safety_leds); i++) {
			const double *color = NULL;
			if (ctx->safety.status == synapse_pb_Safety_Status_SAFETY_SAFE) {
				color = color_safe;
			} else if (ctx->safety.status == synapse_pb_Safety_Status_SAFETY_UNSAFE) {
				color = color_unsafe;
			} else {
				color = color_unknown;
			}
			set_led(safety_leds[i], color, brightness,
				&ctx->led_array.led[led_msg_index]);
			led_msg_index++;
		}

		// headlight leds
		bool lights_on = ctx->status.flag & synapse_pb_Status_Flag_FLAG_LIGHTING;

		if (lights_on) {
			for (size_t i = 0; i < ARRAY_SIZE(headlight_leds); i++) {
				set_led(headlight_leds[i], color_white, 255,
					&ctx->led_array.led[led_msg_index]);
				led_msg_index++;
			}
		} else if (!lights_on) {
			for (size_t i = 0; i < ARRAY_SIZE(headlight_leds); i++) {
				set_led(headlight_leds[i], color_white, 0,
					&ctx->led_array.led[led_msg_index]);
				led_msg_index++;
			}
		}

		// set timestamp
		stamp_msg(&ctx->led_array.stamp, k_uptime_ticks());
		ctx->led_array.led_count = led_msg_index;

		zros_pub_update(&ctx->pub_led_array);
	}

	b3rb_lighting_fini(ctx);
}

static int start(struct context *ctx)
{
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				b3rb_lighting_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "rdd2_lighting");
	k_thread_start(tid);
	return 0;
}

static int b3rb_lighting_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;
	if (argc != 1) {
		LOG_ERR("must have one argument");
		return -1;
	}

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

SHELL_SUBCMD_DICT_SET_CREATE(sub_b3rb_lighting, b3rb_lighting_cmd_handler, (start, &g_ctx, "start"),
			     (stop, &g_ctx, "stop"), (status, &g_ctx, "status"));

SHELL_CMD_REGISTER(b3rb_lighting, &sub_b3rb_lighting, "b3rb lighting commands", NULL);

static int b3rb_lighting_sys_init(void)
{
	return start(&g_ctx);
};

SYS_INIT(b3rb_lighting_sys_init, APPLICATION, 10);

// vi: ts=4 sw=4 et
