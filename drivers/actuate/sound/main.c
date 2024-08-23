/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "actuator_sound.h"
#include <zephyr/drivers/pwm.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#define MY_STACK_SIZE 4096
#define MY_PRIORITY   4

LOG_MODULE_REGISTER(cerebri_actuate_sound, CONFIG_CEREBRI_ACTUATE_SOUND_LOG_LEVEL);

typedef struct _context {
	struct zros_node node;
	synapse_pb_Status status;
	synapse_pb_Status_Mode status_last_mode;
	synapse_pb_Status_Arming status_last_arming;
	synapse_pb_Status_Safety status_last_safety;
	uint32_t status_last_request_seq;
	struct zros_sub sub_status;
	struct tones_t *sound;
	int sound_size;
	const struct pwm_dt_spec buzzer;
	bool started;
} context;

static context g_ctx = {
	.node = {},
	.status = synapse_pb_Status_init_default,
	.status_last_mode = synapse_pb_Status_Mode_MODE_UNKNOWN,
	.status_last_arming = synapse_pb_Status_Arming_ARMING_UNKNOWN,
	.status_last_safety = synapse_pb_Status_Safety_SAFETY_UNKNOWN,
	.status_last_request_seq = 0,
	.sub_status = {},
	.sound = NULL,
	.sound_size = 0,
	.buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer)),
	.started = false,
};

static void init_actuate_sound(context *ctx)
{
	LOG_DBG("init actuate sound");
	zros_node_init(&ctx->node, "actuate_sound");
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);
	if (!pwm_is_ready_dt(&ctx->buzzer)) {
		LOG_ERR("Sound device %s is not ready!", ctx->buzzer.dev->name);
	}
}

static int play_sound(context *ctx, struct tones_t *sound, size_t sound_size)
{
	int err;
	for (size_t i = 0; i < sound_size; i++) {
		if (sound[i].note == REST) {
			err = pwm_set_pulse_dt(&ctx->buzzer, 0);
		} else {
			err = pwm_set_dt(&ctx->buzzer, PWM_HZ(sound[i].note),
					 PWM_HZ((sound[i].note)) / 2);
		}
		if (err) {
			return err;
		}
		k_msleep(sound[i].duration);
	}
	err = pwm_set_pulse_dt(&ctx->buzzer, 0);
	return err;
}

static void actuate_sound_entry_point(void *p0, void *p1, void *p2)
{
	LOG_INF("init");
	context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	init_actuate_sound(ctx);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_status),
	};

	int64_t input_loss_last_alarm_ticks = 0;
	int64_t fuel_low_last_alarm_ticks = 0;
	float input_loss_period_sec = 3.0;
	float fuel_low_period_sec = 10.0;

	while (true) {

		int rc = 0;
		rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
		if (rc != 0) {
			LOG_DBG("Sound poll failed");
			continue;
		}

		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}

		if (ctx->status.mode != ctx->status_last_mode) {
			ctx->status_last_mode = ctx->status.mode;
			if (ctx->status.mode == synapse_pb_Status_Mode_MODE_ACTUATORS) {
				play_sound(ctx, manual_mode_tone, ARRAY_SIZE(manual_mode_tone));
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_BEZIER) {
				play_sound(ctx, auto_mode_tone, ARRAY_SIZE(auto_mode_tone));
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_VELOCITY) {
				play_sound(ctx, cmd_vel_mode_tone, ARRAY_SIZE(cmd_vel_mode_tone));
			} else if (ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION) {
				play_sound(ctx, cal_mode_tone, ARRAY_SIZE(cal_mode_tone));
			}
		}

		if (ctx->status_last_arming == synapse_pb_Status_Arming_ARMING_UNKNOWN) {
			ctx->status_last_arming = ctx->status.arming;
		}

		else if (ctx->status_last_arming == synapse_pb_Status_Arming_ARMING_DISARMED &&
			 ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) {
			ctx->status_last_arming = ctx->status.arming;
			play_sound(ctx, armed_tone, ARRAY_SIZE(armed_tone));
		}

		else if (ctx->status_last_arming == synapse_pb_Status_Arming_ARMING_ARMED &&
			 ctx->status.arming == synapse_pb_Status_Arming_ARMING_DISARMED) {
			ctx->status_last_arming = ctx->status.arming;
			play_sound(ctx, disarmed_tone, ARRAY_SIZE(disarmed_tone));
		}

		if (ctx->status.safety != ctx->status_last_safety) {
			ctx->status_last_safety = ctx->status.safety;
			if (ctx->status.safety == synapse_pb_Status_Safety_SAFETY_SAFE) {
				if (!ctx->started) {
					play_sound(ctx, airy_start_tone,
						   ARRAY_SIZE(airy_start_tone));
					ctx->started = true;
				} else {
					play_sound(ctx, safety_on_tone, ARRAY_SIZE(safety_on_tone));
				}
			}

			else if (ctx->status.safety == synapse_pb_Status_Safety_SAFETY_UNSAFE) {
				play_sound(ctx, safety_off_tone, ARRAY_SIZE(safety_off_tone));
			}
		}

		if (ctx->status.fuel == synapse_pb_Status_Fuel_FUEL_LOW) {
			int64_t now_ticks = k_uptime_ticks();
			if ((now_ticks - fuel_low_last_alarm_ticks) >
			    fuel_low_period_sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
				fuel_low_last_alarm_ticks = now_ticks;
				play_sound(ctx, fuel_tone, ARRAY_SIZE(fuel_tone));
			}
		}

		if (ctx->status.fuel == synapse_pb_Status_Fuel_FUEL_CRITICAL) {
			play_sound(ctx, fuel_tone, ARRAY_SIZE(fuel_tone));
		}

		if (ctx->status.input_status == synapse_pb_Status_LinkStatus_STATUS_LOSS &&
		    ctx->status.safety == synapse_pb_Status_Safety_SAFETY_UNSAFE) {
			int64_t now_ticks = k_uptime_ticks();
			if ((now_ticks - input_loss_last_alarm_ticks) >
			    input_loss_period_sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) {
				input_loss_last_alarm_ticks = now_ticks;
				play_sound(ctx, input_loss_tone, ARRAY_SIZE(input_loss_tone));
			}
		}

		if (ctx->status.request_rejected &&
		    ctx->status_last_request_seq != ctx->status.request_seq) {
			ctx->status_last_request_seq = ctx->status.request_seq;
			play_sound(ctx, reject_tone, ARRAY_SIZE(reject_tone));
		}
	}
}

K_THREAD_DEFINE(actuate_sound, MY_STACK_SIZE, actuate_sound_entry_point, &g_ctx, NULL, NULL,
		MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
