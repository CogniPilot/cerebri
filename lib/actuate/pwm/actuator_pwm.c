/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuator_pwm.h"
#include <stdio.h>
#include <zephyr/drivers/pwm.h>

#define PWM_SHELL_NODE DT_NODE_EXISTS(DT_NODELABEL(pwm_shell))

#if PWM_SHELL_NODE

actuator_pwm_t g_actuator_pwms[] = {
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 0
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_0,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_0,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_0,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_0
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_0,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_0
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_0
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_0
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_0) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_0,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_0) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_0,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_0,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux0)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 1
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_1,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_1,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_1,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_1
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_1,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_1
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_1
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_1
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_1) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_1,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_1) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_1,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_1,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux1)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 2
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_2,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_2,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_2,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_2
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_2,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_2
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_2
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_2
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_2) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_2,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_2) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_2,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_2,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux2)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 3
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_3,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_3,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_3,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_3
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_3,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_3
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_3
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_3
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_3) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_3,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_3) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_3,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_3,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux3)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 4
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_4,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_4,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_4,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_4
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_4,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_4
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_4
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_4
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_4) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_4,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_4) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_4,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_4,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux4)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 5
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_5,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_5,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_5,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_5
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_5,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_5
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_5
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_5
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_5) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_5,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_5) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_5,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_5,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux5)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 6
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_6,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_6,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_6,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_6
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_6,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_6
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_6
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_6
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_6) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_6,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_6) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_6,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_6,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux6)),
    },
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_NUMBER > 7
    {
        .min = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MIN_7,
        .max = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_MAX_7,
        .center = CONFIG_CEREBRI_ACTUATE_PWM_PULSE_CENTER_7,
#if CONFIG_CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_7
        .use_nano_seconds = true,
#else
        .use_nano_seconds = false,
#endif
        .alias = CONFIG_CEREBRI_ACTUATE_PWM_OUTPUT_7,
#if !CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_7
        .type = PWM_TYPE_NORMALIZED,
#elif CONFIG_CEREBRI_ACTUATE_PWM_USE_POS_7
        .type = PWM_TYPE_POSITION,
#else
        .type = PWM_TYPE_VELOCITY,
#endif
#if CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_7
        .slope = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_7) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_7,
        .intercept = ((float)CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_7) / CONFIG_CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_7,
#else
        .slope = 0,
        .intercept = 0,
#endif
        .index = CONFIG_CEREBRI_ACTUATE_PWM_INDEX_7,
        .device = PWM_DT_SPEC_GET(DT_CHILD(DT_NODELABEL(pwm_shell), aux7)),
    },
#endif
};

#endif

/* vi: ts=4 sw=4 et */
