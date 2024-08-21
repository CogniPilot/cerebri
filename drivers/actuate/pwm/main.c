/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cerebri_pwm_actuators

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <cerebri/core/perf_duration.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(actuate_pwm, CONFIG_CEREBRI_ACTUATE_PWM_LOG_LEVEL);

#define CONFIG_PWM_ACTUATORS_INIT_PRIORITY 50
#define MY_STACK_SIZE 4096
#define MY_PRIORITY 4

extern struct perf_duration control_latency;

typedef enum pwm_type_t {
    PWM_TYPE_normalized = 0,
    PWM_TYPE_position = 1,
    PWM_TYPE_velocity = 2,
} pwm_type_t;

typedef struct actuator_pwm_t {
    const char* label;
    struct pwm_dt_spec device;
    uint32_t disarmed;
    uint32_t min;
    uint32_t max;
    uint32_t center;
    bool use_nano_seconds;
    double scale;
    uint8_t index;
    pwm_type_t type;
} actuator_pwm_t;

struct context {
    synapse_pb_Actuators actuators;
    synapse_pb_Status status;
    synapse_pb_Pwm pwm;
    struct zros_node node;
    struct zros_sub sub_actuators, sub_status;
    struct zros_pub pub_pwm;
    struct k_sem running;
    size_t stack_size;
    k_thread_stack_t* stack_area;
    struct k_thread thread_data;
    uint32_t test_pulse;
    uint8_t num_actuators;
    const actuator_pwm_t* actuator_pwms;
};

static int actuate_pwm_init(struct context* ctx)
{
    LOG_INF("init");
    // check pwm config
    for (int i = 0; i < ctx->num_actuators; i++) {
        actuator_pwm_t pwm = ctx->actuator_pwms[i];
        if (pwm.max < pwm.min) {
            LOG_ERR("config pwm_%d min, max must monotonically increase",
                i);
            return -1;
        }
    }

    zros_node_init(&ctx->node, "actuate_pwm");
    zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1000);
    zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
    zros_pub_init(&ctx->pub_pwm, &ctx->node, &topic_pwm, &ctx->pwm);
    k_sem_take(&ctx->running, K_FOREVER);
    return 0;
}

static void actuate_pwm_fini(struct context* ctx)
{
    LOG_INF("fini");
    zros_sub_fini(&ctx->sub_actuators);
    zros_sub_fini(&ctx->sub_status);
    zros_pub_fini(&ctx->pub_pwm);
    zros_node_fini(&ctx->node);
    k_sem_give(&ctx->running);
}

static void pwm_update(struct context* ctx)
{
    bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
    int err = 0;

    for (int i = 0; i < ctx->num_actuators; i++) {
        actuator_pwm_t pwm = ctx->actuator_pwms[i];

        uint32_t pulse = pwm.disarmed;
        double input = 0;

        if (pwm.type == PWM_TYPE_normalized) {
            input = ctx->actuators.normalized[pwm.index];
        } else if (pwm.type == PWM_TYPE_position) {
            input = ctx->actuators.position[pwm.index];
        } else if (pwm.type == PWM_TYPE_velocity) {
            input = ctx->actuators.velocity[pwm.index];
        }

        if (armed) {
            pulse = (uint32_t)((pwm.scale * input) + pwm.center);
            if (pulse > pwm.max) {
                pulse = pwm.max;
                LOG_DBG("%d  pwm saturated, requested %d > %d", pwm.index, pulse, pwm.max);
            } else if (pulse < pwm.min) {
                pulse = pwm.min;
                LOG_DBG("%d  pwm saturated, requested %d < %d", pwm.index, pulse, pwm.min);
            }
        }

        ctx->pwm.channel[i] = pulse;

        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }
        perf_duration_stop(&control_latency);

        if (err) {
            LOG_ERR("Failed to set pulse %d on %d (err %d)", pulse, pwm.index, err);
        }
    }

    stamp_msg(&ctx->pwm.timestamp, k_uptime_ticks());
    zros_pub_update(&ctx->pub_pwm);
}

static void actuate_pwm_run(void* p0, void* p1, void* p2)
{
    struct context* ctx = p0;
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);

    int ret = actuate_pwm_init(ctx);

    if (ret < 0) {
        return;
    }

    struct k_poll_event events[] = {
        *zros_sub_get_event(&ctx->sub_actuators),
    };

    while (k_sem_take(&ctx->running, K_NO_WAIT) < 0) {
        int rc = 0;
        rc = k_poll(events, ARRAY_SIZE(events), K_MSEC(1000));
        if (rc != 0) {
            LOG_DBG("no actuator message received");
            // put motors in disarmed state
            if (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) {
                ctx->status.arming = synapse_pb_Status_Arming_ARMING_DISARMED;
                LOG_ERR("disarming motors due to actuator msg timeout!");
            }
        }

        if (zros_sub_update_available(&ctx->sub_status)) {
            zros_sub_update(&ctx->sub_status);
        }

        if (zros_sub_update_available(&ctx->sub_actuators)) {
            zros_sub_update(&ctx->sub_actuators);
        }

        // update pwm
        pwm_update(ctx);
    }

    actuate_pwm_fini(ctx);
}

static int start(struct context* ctx)
{
    __ASSERT_NO_MSG(ctx != NULL);
    k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area,
        ctx->stack_size,
        actuate_pwm_run,
        ctx, NULL, NULL,
        MY_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(tid, "actuate_pwm");
    k_thread_start(tid);
    return 0;
}

static int set_pulse_all(struct context* ctx, uint32_t pulse)
{
    for (int i = 0; i < ctx->num_actuators; i++) {
        actuator_pwm_t pwm = ctx->actuator_pwms[i];
        int err = 0;
        if (pwm.use_nano_seconds) {
            err = pwm_set_pulse_dt(&pwm.device, PWM_NSEC(pulse));
        } else {
            err = pwm_set_pulse_dt(&pwm.device, PWM_USEC(pulse));
        }
        if (err) {
            LOG_ERR("failed to set pulse %d on %d (err %d)", pwm.max, pwm.index, err);
        }
    }
    return 0;
}

static int actuate_pwm_cmd_handler(const struct shell* sh,
    size_t argc, char** argv, void* data)
{
    ARG_UNUSED(argc);
    struct context* ctx = data;
    __ASSERT_NO_MSG(data != NULL);

    if (strcmp(argv[0], "start") == 0) {
        if (k_sem_count_get(&ctx->running) == 0) {
            shell_print(sh, "already running");
        } else {
            start(ctx);
        }
    } else if (strcmp(argv[0], "stop") == 0) {
        if (k_sem_count_get(&ctx->running) == 0) {
            k_sem_give(&ctx->running);
        } else {
            shell_print(sh, "not running");
        }
    } else if (strcmp(argv[0], "set_all_1000") == 0) {
        if (k_sem_count_get(&ctx->running) == 0) {
            shell_print(sh, "must stop before using set");
        } else {
            set_pulse_all(ctx, 1000);
        }
    } else if (strcmp(argv[0], "set_all_1500") == 0) {
        if (k_sem_count_get(&ctx->running) == 0) {
            shell_print(sh, "must stop before using set");
        } else {
            set_pulse_all(ctx, 1500);
        }
    } else if (strcmp(argv[0], "set_all_2000") == 0) {
        if (k_sem_count_get(&ctx->running) == 0) {
            shell_print(sh, "must stop before using set");
        } else {
            set_pulse_all(ctx, 2000);
        }
    } else if (strcmp(argv[0], "status") == 0) {
        shell_print(sh, "running: %d", (int)k_sem_count_get(&ctx->running) == 0);
    } else {
        shell_print(sh, "unknown command");
    }
    return 0;
}

static int actuate_pwm_device_init(const struct device* dev)
{
    // const struct blink_gpio_led_config *config = dev->config;
    struct context* data = dev->data;
    start(data);
    return 0;
}

#define PWM_ACTUATOR_SHELL(inst)                                                  \
    SHELL_SUBCMD_DICT_SET_CREATE(sub_actuate_pwm_##inst, actuate_pwm_cmd_handler, \
        (start, &data_##inst, "start"),                                           \
        (stop, &data_##inst, "stop"),                                             \
        (status, &data_##inst, "status"),                                         \
        (set_all_1000, &data_##inst, "set_all_1000"),                             \
        (set_all_1500, &data_##inst, "set_all_1500"),                             \
        (set_all_2000, &data_##inst, "set_all_2000"));                            \
    SHELL_CMD_REGISTER(actuate_pwm_##inst, &sub_actuate_pwm_##inst, "actuate_pwm commands", NULL);

#define PWM_ACTUATOR_DEFINE(node_id)                                              \
    {                                                                             \
        .label = DT_NODE_FULL_NAME(node_id),                                      \
        .device = PWM_DT_SPEC_GET_BY_IDX(node_id, 0),                             \
        .disarmed = DT_PROP(node_id, disarmed),                                   \
        .min = DT_PROP(node_id, min),                                             \
        .max = DT_PROP(node_id, max),                                             \
        .center = DT_PROP(node_id, center),                                       \
        .use_nano_seconds = DT_PROP(node_id, use_nano_seconds),                   \
        .scale = ((double)DT_PROP(node_id, scale)) / DT_PROP(node_id, scale_div), \
        .index = DT_PROP(node_id, input_index),                                   \
        .type = DT_ENUM_IDX(node_id, input_type),                                 \
    },

#define PWM_ACTUATORS_DEFINE(inst)                                                  \
    static const actuator_pwm_t g_actuator_pwms_##inst[] = {                        \
        DT_FOREACH_CHILD(DT_INST(inst, cerebri_pwm_actuators), PWM_ACTUATOR_DEFINE) \
    };                                                                              \
    static K_THREAD_STACK_DEFINE(g_my_stack_area_##inst, MY_STACK_SIZE);            \
    static struct context data_##inst = {                                           \
        .actuators = synapse_pb_Actuators_init_default,                             \
        .status = synapse_pb_Status_init_default,                                   \
        .pwm = {                                                                    \
            .has_timestamp = true,                                                  \
            .channel = {},                                                          \
            .channel_count = DT_CHILD_NUM(DT_INST(inst, cerebri_pwm_actuators)),    \
        },                                                                          \
        .node = {},                                                                 \
        .sub_status = {},                                                           \
        .sub_actuators = {},                                                        \
        .pub_pwm = {},                                                              \
        .running = Z_SEM_INITIALIZER(data_##inst.running, 1, 1),                    \
        .stack_size = MY_STACK_SIZE,                                                \
        .stack_area = g_my_stack_area_##inst,                                       \
        .thread_data = {},                                                          \
        .test_pulse = 0,                                                            \
        .actuator_pwms = g_actuator_pwms_##inst,                                    \
        .num_actuators = DT_CHILD_NUM(DT_INST(inst, cerebri_pwm_actuators)),        \
    };                                                                              \
    PWM_ACTUATOR_SHELL(inst);                                                       \
    DEVICE_DT_INST_DEFINE(inst, actuate_pwm_device_init, NULL, &data_##inst,        \
        NULL, POST_KERNEL,                                                          \
        CONFIG_PWM_ACTUATORS_INIT_PRIORITY,                                         \
        NULL);

DT_INST_FOREACH_STATUS_OKAY(PWM_ACTUATORS_DEFINE)
