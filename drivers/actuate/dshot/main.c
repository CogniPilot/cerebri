/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cerebri_dshot_actuators

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
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

LOG_MODULE_REGISTER(actuate_dshot, CONFIG_CEREBRI_ACTUATE_DSHOT_LOG_LEVEL);

#define CONFIG_DSHOT_ACTUATORS_INIT_PRIORITY 50
#define MY_STACK_SIZE                        4096
#define MY_PRIORITY                          4

extern struct perf_duration control_latency;

typedef enum dshot_type_t {
	DSHOT_TYPE_normalized = 0,
	DSHOT_TYPE_velocity = 1,
} dshot_type_t;

typedef struct actuator_dshot_t {
	const char *label;
	uint32_t disarmed;
	uint32_t center;
	double scale;
	uint8_t index;
	dshot_type_t type;
} actuator_dshot_t;

struct context {
	synapse_pb_Actuators actuators;
	synapse_pb_Status status;
	struct zros_node node;
	struct zros_sub sub_actuators, sub_status;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	const struct device *const dev;
	uint8_t num_actuators;
	const actuator_dshot_t *dshot_actuators;
};

static int actuate_dshot_init(struct context *ctx)
{
	LOG_INF("init");
	if (ctx->num_actuators != nxp_flexio_dshot_channel_count(ctx->dev)) {
		LOG_ERR("dshot actuator config mismatch! %i %i", ctx->num_actuators,
			nxp_flexio_dshot_channel_count(ctx->dev));
	}
	zros_node_init(&ctx->node, "actuate_dshot");
	zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1000);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	k_sem_take(&ctx->running, K_FOREVER);
	return 0;
}

static void actuate_dshot_fini(struct context *ctx)
{
	LOG_INF("fini");
	zros_sub_fini(&ctx->sub_actuators);
	zros_sub_fini(&ctx->sub_status);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
}

static void dshot_update(struct context *ctx)
{
	bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;

	for (int i = 0; i < ctx->num_actuators; i++) {
		actuator_dshot_t dshot = ctx->dshot_actuators[i];

		double input = 0;
		uint32_t throttle = 0;

		if (armed) {
			input = ctx->actuators.velocity[i];
			throttle = (uint32_t)((dshot.scale * input) + dshot.center);

			if (throttle > DSHOT_MAX) {
				throttle = DSHOT_MAX;
			} else if (throttle < DSHOT_MIN) {
				throttle = DSHOT_MIN;
			}
		}

		nxp_flexio_dshot_data_set(ctx->dev, i, (uint16_t)throttle, false);

		perf_duration_stop(&control_latency);
	}

	nxp_flexio_dshot_trigger(ctx->dev);
}

static void dshot_beep(const struct shell *sh, struct context *ctx, int motor)
{
	if (motor < ctx->num_actuators) {
		nxp_flexio_dshot_data_set(ctx->dev, motor, DSHOT_CMD_BEEP1, false);
		nxp_flexio_dshot_trigger(ctx->dev);

		// wait 260 ms
		k_msleep(260);

		shell_print(sh, "beep: %i", motor);
	} else {
		shell_print(sh, "dshot-actuator %i doesn't exist", motor);
	}
}

static void dshot_spin_dir(const struct shell *sh, struct context *ctx, int motor, bool reverse)
{

	if (motor < ctx->num_actuators) {
		uint16_t dir;

		if (!reverse) {
			dir = DSHOT_CMD_SPIN_DIRECTION_1;
			shell_print(sh, "actuator: %i normal direction", motor);
		} else {
			dir = DSHOT_CMD_SPIN_DIRECTION_2;
			shell_print(sh, "actuator: %i reverse direction", motor);
		}
		for (int i = 0; i < 6; i++) {
			nxp_flexio_dshot_data_set(ctx->dev, motor, dir, false);
			nxp_flexio_dshot_trigger(ctx->dev);
		}

		nxp_flexio_dshot_data_set(ctx->dev, motor, DSHOT_CMD_SAVE_SETTINGS, false);
		nxp_flexio_dshot_trigger(ctx->dev);

		// wait 35 ms
		k_msleep(35);

	} else {
		shell_print(sh, "dshot-actuator %i doesn't exist", motor);
	}
}

static void actuate_dshot_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = actuate_dshot_init(ctx);

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

		// update dshot
		dshot_update(ctx);
	}

	actuate_dshot_fini(ctx);
}

static int start(struct context *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				actuate_dshot_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "actuate_dshot");
	k_thread_start(tid);
	return 0;
}

static int actuate_dshot_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
{
	ARG_UNUSED(argc);
	struct context *ctx = data;
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
	} else if (strcmp(argv[0], "status") == 0) {
		shell_print(sh, "running: %d", (int)k_sem_count_get(&ctx->running) == 0);
	} else if (strcmp(argv[0], "beep") == 0) {
		if (k_sem_count_get(&ctx->running) == 0) {
			shell_print(sh, "must stop before using set");
		} else {
			dshot_beep(sh, ctx, atoi(argv[1]));
		}
	} else if (strcmp(argv[0], "dir") == 0) {
		if (k_sem_count_get(&ctx->running) == 0) {
			shell_print(sh, "must stop before using set");
		} else {
			if (strcmp(argv[2], "normal") == 0) {
				dshot_spin_dir(sh, ctx, atoi(argv[1]), false);
			} else if (strcmp(argv[2], "reverse") == 0) {
				dshot_spin_dir(sh, ctx, atoi(argv[1]), true);
			} else {
				shell_print(sh, "wrong argument");
			}
		}
	} else {
		shell_print(sh, "unknown command");
	}
	return 0;
}

static int actuate_dshot_device_init(const struct device *dev)
{
	struct context *data = dev->data;
	start(data);
	return 0;
}

#define DSHOT_ACTUATOR_SHELL(inst)                                                                 \
	static int actuate_dshot_cmd_handler_##inst(const struct shell *sh, size_t argc,           \
						    char **argv, void *data)                       \
	{                                                                                          \
		return actuate_dshot_cmd_handler(sh, argc, argv, (void *)&data_##inst);            \
	}                                                                                          \
	SHELL_STATIC_SUBCMD_SET_CREATE(                                                            \
		sub_actuate_dshot_##inst, /* Alphabetically sorted. */                             \
		SHELL_CMD(start, NULL, "start", actuate_dshot_cmd_handler_##inst),                 \
		SHELL_CMD(stop, NULL, "stop", actuate_dshot_cmd_handler_##inst),                   \
		SHELL_CMD(status, NULL, "status", actuate_dshot_cmd_handler_##inst),               \
		SHELL_CMD_ARG(beep, NULL, "beep <actuator_index>",                                 \
			      actuate_dshot_cmd_handler_##inst, 2, 0),                             \
		SHELL_CMD_ARG(dir, NULL,                                                           \
			      "dir <actuator_index> \nchange motor spin direction: "               \
			      "\"normal\" or \"reverse\"",                                         \
			      actuate_dshot_cmd_handler_##inst, 3, 0),                             \
		SHELL_SUBCMD_SET_END /* Array terminated. */                                       \
	);                                                                                         \
	SHELL_CMD_REGISTER(actuate_dshot_##inst, &sub_actuate_dshot_##inst,                        \
			   "actuate_dshot commands", NULL);

#define DSHOT_ACTUATOR_DEFINE(node_id)                                                             \
	{                                                                                          \
		.label = DT_NODE_FULL_NAME(node_id),                                               \
		.center = DT_PROP(node_id, center),                                                \
		.scale = ((double)DT_PROP(node_id, scale)) / DT_PROP(node_id, scale_div),          \
		.index = DT_PROP(node_id, input_index),                                            \
		.type = DT_ENUM_IDX(node_id, input_type),                                          \
	},

#define DSHOT_ACTUATORS_DEFINE(inst)                                                               \
	static const actuator_dshot_t g_actuator_dshots_##inst[] = {                               \
		DT_FOREACH_CHILD(DT_INST(inst, cerebri_dshot_actuators), DSHOT_ACTUATOR_DEFINE)};  \
	static K_THREAD_STACK_DEFINE(g_my_stack_area_##inst, MY_STACK_SIZE);                       \
	static struct context data_##inst = {                                                      \
		.actuators = synapse_pb_Actuators_init_default,                                    \
		.status = synapse_pb_Status_init_default,                                          \
		.dev = DEVICE_DT_GET(DT_NODELABEL(dshot)),                                         \
		.node = {},                                                                        \
		.sub_status = {},                                                                  \
		.sub_actuators = {},                                                               \
		.running = Z_SEM_INITIALIZER(data_##inst.running, 1, 1),                           \
		.stack_size = MY_STACK_SIZE,                                                       \
		.stack_area = g_my_stack_area_##inst,                                              \
		.thread_data = {},                                                                 \
		.dshot_actuators = g_actuator_dshots_##inst,                                       \
		.num_actuators = DT_CHILD_NUM(DT_INST(inst, cerebri_dshot_actuators)),             \
	};                                                                                         \
	DSHOT_ACTUATOR_SHELL(inst);                                                                \
	DEVICE_DT_INST_DEFINE(inst, actuate_dshot_device_init, NULL, &data_##inst, NULL,           \
			      POST_KERNEL, CONFIG_DSHOT_ACTUATORS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DSHOT_ACTUATORS_DEFINE)