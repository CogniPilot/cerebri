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

/* Command types for pending DShot special commands */
typedef enum dshot_cmd_type_t {
	DSHOT_CMD_TYPE_NONE = 0,
	DSHOT_CMD_TYPE_BEEP,
	DSHOT_CMD_TYPE_SPIN_DIR,
} dshot_cmd_type_t;

/* Steps for multi-phase commands */
typedef enum dshot_cmd_step_t {
	DSHOT_CMD_STEP_SEND = 0, /* Sending command frames */
	DSHOT_CMD_STEP_SAVE,     /* Sending save settings (for spin_dir) */
	DSHOT_CMD_STEP_DELAY,    /* Post-command delay */
	DSHOT_CMD_STEP_DONE,     /* Command complete */
} dshot_cmd_step_t;

/* Pending command state */
struct dshot_pending_cmd {
	dshot_cmd_type_t type;
	int8_t motor;          /* Target motor index */
	uint8_t param;         /* Command parameter (e.g., direction) */
	dshot_cmd_step_t step; /* Current step in command sequence */
	uint8_t count;         /* Repetitions remaining for current step */
	int64_t delay_until;   /* Timestamp (ms) when delay ends */
};

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
	struct dshot_pending_cmd pending_cmd;
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

/* Process pending command state machine, returns true if command is active */
static bool dshot_process_command(struct context *ctx)
{
	struct dshot_pending_cmd *cmd = &ctx->pending_cmd;

	if (cmd->type == DSHOT_CMD_TYPE_NONE) {
		return false;
	}

	int64_t now = k_uptime_get();

	/* Handle delay step */
	if (cmd->step == DSHOT_CMD_STEP_DELAY) {
		if (now >= cmd->delay_until) {
			cmd->step = DSHOT_CMD_STEP_DONE;
		} else {
			/* Still in delay, send disarmed to all */
			for (int i = 0; i < ctx->num_actuators; i++) {
				nxp_flexio_dshot_data_set(ctx->dev, i, DSHOT_DISARMED, false);
			}
			nxp_flexio_dshot_trigger(ctx->dev);
			return true;
		}
	}

	/* Handle done step */
	if (cmd->step == DSHOT_CMD_STEP_DONE) {
		LOG_INF("command complete: type=%d motor=%d", cmd->type, cmd->motor);
		cmd->type = DSHOT_CMD_TYPE_NONE;
		return false;
	}

	/* Determine what command value to send based on type and step */
	uint16_t cmd_value = DSHOT_DISARMED;

	if (cmd->type == DSHOT_CMD_TYPE_BEEP) {
		if (cmd->step == DSHOT_CMD_STEP_SEND) {
			cmd_value = DSHOT_CMD_BEEP1;
		}
	} else if (cmd->type == DSHOT_CMD_TYPE_SPIN_DIR) {
		if (cmd->step == DSHOT_CMD_STEP_SEND) {
			cmd_value = cmd->param; /* Direction command */
		} else if (cmd->step == DSHOT_CMD_STEP_SAVE) {
			cmd_value = DSHOT_CMD_SAVE_SETTINGS;
		}
	}

	/* Send command to target motor, disarmed to others */
	for (int i = 0; i < ctx->num_actuators; i++) {
		if (i == cmd->motor) {
			nxp_flexio_dshot_data_set(ctx->dev, i, cmd_value, true);
		} else {
			nxp_flexio_dshot_data_set(ctx->dev, i, DSHOT_DISARMED, false);
		}
	}
	nxp_flexio_dshot_trigger(ctx->dev);

	/* Decrement count and advance step when done */
	cmd->count--;
	if (cmd->count == 0) {
		if (cmd->type == DSHOT_CMD_TYPE_BEEP) {
			/* Beep: after sending, go to delay (380ms per BLHeli spec) */
			cmd->step = DSHOT_CMD_STEP_DELAY;
			cmd->delay_until = now + 380;
		} else if (cmd->type == DSHOT_CMD_TYPE_SPIN_DIR) {
			if (cmd->step == DSHOT_CMD_STEP_SEND) {
				/* After direction, send save settings 6x */
				cmd->step = DSHOT_CMD_STEP_SAVE;
				cmd->count = 6;
			} else if (cmd->step == DSHOT_CMD_STEP_SAVE) {
				/* After save, delay 35ms */
				cmd->step = DSHOT_CMD_STEP_DELAY;
				cmd->delay_until = now + 35;
			}
		}
	}

	return true;
}

static void dshot_update(struct context *ctx)
{
	bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;

	/* If armed, ignore any pending commands and send throttle */
	if (armed) {
		ctx->pending_cmd.type = DSHOT_CMD_TYPE_NONE;

		for (int i = 0; i < ctx->num_actuators; i++) {
			actuator_dshot_t dshot = ctx->dshot_actuators[i];
			double input = ctx->actuators.velocity[i];
			uint32_t throttle = (uint32_t)((dshot.scale * input) + dshot.center);

			if (throttle > DSHOT_MAX) {
				throttle = DSHOT_MAX;
			} else if (throttle < DSHOT_MIN) {
				throttle = DSHOT_MIN;
			}

			nxp_flexio_dshot_data_set(ctx->dev, i, (uint16_t)throttle, false);
			perf_duration_stop(&control_latency);
		}
		nxp_flexio_dshot_trigger(ctx->dev);
		return;
	}

	/* Not armed: process pending command or send disarmed */
	if (dshot_process_command(ctx)) {
		return;
	}

	/* No command pending, send disarmed to all */
	for (int i = 0; i < ctx->num_actuators; i++) {
		nxp_flexio_dshot_data_set(ctx->dev, i, DSHOT_DISARMED, false);
	}
	nxp_flexio_dshot_trigger(ctx->dev);
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
		if (ctx->pending_cmd.type != DSHOT_CMD_TYPE_NONE) {
			shell_print(sh, "pending cmd: type=%d motor=%d step=%d",
				    ctx->pending_cmd.type, ctx->pending_cmd.motor,
				    ctx->pending_cmd.step);
		}
	} else if (strcmp(argv[0], "beep") == 0) {
		int motor = atoi(argv[1]);
		if (motor >= ctx->num_actuators) {
			shell_print(sh, "actuator %d doesn't exist", motor);
		} else if (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) {
			shell_print(sh, "cannot beep while armed");
		} else if (ctx->pending_cmd.type != DSHOT_CMD_TYPE_NONE) {
			shell_print(sh, "command already pending");
		} else {
			ctx->pending_cmd.type = DSHOT_CMD_TYPE_BEEP;
			ctx->pending_cmd.motor = motor;
			ctx->pending_cmd.step = DSHOT_CMD_STEP_SEND;
			ctx->pending_cmd.count = 10;
			shell_print(sh, "beep queued: motor %d", motor);
		}
	} else if (strcmp(argv[0], "dir") == 0) {
		int motor = atoi(argv[1]);
		if (motor >= ctx->num_actuators) {
			shell_print(sh, "actuator %d doesn't exist", motor);
		} else if (ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED) {
			shell_print(sh, "cannot change direction while armed");
		} else if (ctx->pending_cmd.type != DSHOT_CMD_TYPE_NONE) {
			shell_print(sh, "command already pending");
		} else if (strcmp(argv[2], "normal") == 0) {
			ctx->pending_cmd.type = DSHOT_CMD_TYPE_SPIN_DIR;
			ctx->pending_cmd.motor = motor;
			ctx->pending_cmd.param = DSHOT_CMD_SPIN_DIRECTION_1;
			ctx->pending_cmd.step = DSHOT_CMD_STEP_SEND;
			ctx->pending_cmd.count = 6;
			shell_print(sh, "direction queued: motor %d normal", motor);
		} else if (strcmp(argv[2], "reverse") == 0) {
			ctx->pending_cmd.type = DSHOT_CMD_TYPE_SPIN_DIR;
			ctx->pending_cmd.motor = motor;
			ctx->pending_cmd.param = DSHOT_CMD_SPIN_DIRECTION_2;
			ctx->pending_cmd.step = DSHOT_CMD_STEP_SEND;
			ctx->pending_cmd.count = 6;
			shell_print(sh, "direction queued: motor %d reverse", motor);
		} else {
			shell_print(sh, "wrong argument: use 'normal' or 'reverse'");
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
		.pending_cmd = {.type = DSHOT_CMD_TYPE_NONE},                                      \
	};                                                                                         \
	DSHOT_ACTUATOR_SHELL(inst);                                                                \
	DEVICE_DT_INST_DEFINE(inst, actuate_dshot_device_init, NULL, &data_##inst, NULL,           \
			      POST_KERNEL, CONFIG_DSHOT_ACTUATORS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DSHOT_ACTUATORS_DEFINE)
