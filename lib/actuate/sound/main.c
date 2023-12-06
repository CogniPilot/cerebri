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
#define MY_PRIORITY 4

LOG_MODULE_REGISTER(cerebri_actuate_sound, CONFIG_CEREBRI_ACTUATE_SOUND_LOG_LEVEL);

typedef struct _context {
    struct zros_node node;
    synapse_msgs_Fsm fsm;
    synapse_msgs_Fsm_Mode fsm_last_mode;
    synapse_msgs_Fsm_Armed fsm_last_armed;
    synapse_msgs_BatteryState battery_state;
    synapse_msgs_Safety safety;
    synapse_msgs_Safety_Status safety_last_status;
    struct zros_sub sub_fsm, sub_battery_state, sub_safety;
    struct tones_t* sound;
    int sound_size;
    const struct pwm_dt_spec buzzer;
    bool started;
} context;

static context g_ctx = {
    .node = {},
    .fsm = synapse_msgs_Fsm_init_default,
    .fsm_last_mode = synapse_msgs_Fsm_Mode_UNKNOWN_MODE,
    .fsm_last_armed = synapse_msgs_Fsm_Armed_UNKNOWN_ARMING,
    .battery_state = synapse_msgs_BatteryState_init_default,
    .safety_last_status = synapse_msgs_Safety_Status_UNKNOWN,
    .safety = synapse_msgs_Safety_init_default,
    .sub_fsm = {},
    .sub_battery_state = {},
    .sub_safety = {},
    .sound = NULL,
    .sound_size = 0,
    .buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer)),
    .started = false,
};

static void init_actuate_sound(context* ctx)
{
    LOG_DBG("init actuate sound");
    zros_node_init(&ctx->node, "actuate_sound");
    zros_sub_init(&ctx->sub_fsm, &ctx->node, &topic_fsm, &ctx->fsm, 1);
    zros_sub_init(&ctx->sub_battery_state, &ctx->node, &topic_battery_state, &ctx->battery_state, 1);
    zros_sub_init(&ctx->sub_safety, &ctx->node, &topic_safety, &ctx->safety, 1);
    if (!pwm_is_ready_dt(&ctx->buzzer)) {
        LOG_ERR("Sound device %s is not ready!", ctx->buzzer.dev->name);
    }
}

static int play_sound(context* ctx, struct tones_t* sound, size_t sound_size)
{
    for (size_t i = 0; i < sound_size; i++) {
        if (sound[i].note == REST) {
            pwm_set_pulse_dt(&ctx->buzzer, 0);
        } else {
            pwm_set_dt(&ctx->buzzer, PWM_HZ(sound[i].note), PWM_HZ((sound[i].note)) / 2);
        }
        k_msleep(sound[i].duration);
    }
    pwm_set_pulse_dt(&ctx->buzzer, 0);
    return 0;
}

static void actuate_sound_entry_point(void* p0, void* p1, void* p2)
{
    context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    init_actuate_sound(ctx);

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_fsm),
        *zros_sub_get_event(&ctx->sub_battery_state),
        *zros_sub_get_event(&ctx->sub_safety),
    };

    while (true) {

        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("Sound poll failed");
            continue;
        }

        if (zros_sub_update_available(&ctx->sub_fsm)) {
            zros_sub_update(&ctx->sub_fsm);
        }

        if (zros_sub_update_available(&ctx->sub_battery_state)) {
            zros_sub_update(&ctx->sub_battery_state);
        }

        if (zros_sub_update_available(&ctx->sub_safety)) {
            zros_sub_update(&ctx->sub_safety);
        }

        if (ctx->fsm.mode != ctx->fsm_last_mode) {
            ctx->fsm_last_mode = ctx->fsm.mode;
            if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_MANUAL) {
                play_sound(ctx, manual_mode_tone, ARRAY_SIZE(manual_mode_tone));
            } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_AUTO) {
                play_sound(ctx, auto_mode_tone, ARRAY_SIZE(auto_mode_tone));
            } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_CMD_VEL) {
                play_sound(ctx, cmd_vel_mode_tone, ARRAY_SIZE(cmd_vel_mode_tone));
            } else if (ctx->fsm.mode == synapse_msgs_Fsm_Mode_CALIBRATION) {
                play_sound(ctx, cal_mode_tone, ARRAY_SIZE(cal_mode_tone));
            }
        }

        if (ctx->fsm_last_armed == synapse_msgs_Fsm_Armed_UNKNOWN_ARMING) {
            ctx->fsm_last_armed = ctx->fsm.armed;
        }

        else if (ctx->fsm_last_armed == synapse_msgs_Fsm_Armed_DISARMED && ctx->fsm.armed == synapse_msgs_Fsm_Armed_ARMED) {
            ctx->fsm_last_armed = ctx->fsm.armed;
            play_sound(ctx, armed_tone, ARRAY_SIZE(armed_tone));
        }

        else if (ctx->fsm_last_armed == synapse_msgs_Fsm_Armed_ARMED && ctx->fsm.armed == synapse_msgs_Fsm_Armed_DISARMED) {
            ctx->fsm_last_armed = ctx->fsm.armed;
            play_sound(ctx, disarmed_tone, ARRAY_SIZE(disarmed_tone));
        }

        if (ctx->safety.status != ctx->safety_last_status) {
            ctx->safety_last_status = ctx->safety.status;
            if (ctx->safety.status == synapse_msgs_Safety_Status_SAFE) {
                if (!ctx->started) {
                    play_sound(ctx, airy_start_tone, ARRAY_SIZE(airy_start_tone));
                    ctx->started = true;
                } else {
                    play_sound(ctx, safety_on_tone, ARRAY_SIZE(safety_on_tone));
                }
            }

            else if (ctx->safety.status == synapse_msgs_Safety_Status_UNSAFE) {
                play_sound(ctx, safety_off_tone, ARRAY_SIZE(safety_off_tone));
            }
        }

        if (ctx->battery_state.voltage < CONFIG_CEREBRI_B3RB_BATTERY_MIN_MILLIVOLT / 1000.0) {
            play_sound(ctx, critical_tone, ARRAY_SIZE(critical_tone));
        }
    }
}

K_THREAD_DEFINE(actuate_sound, MY_STACK_SIZE,
    actuate_sound_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
