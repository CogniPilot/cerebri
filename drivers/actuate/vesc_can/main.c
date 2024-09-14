/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cerebri_vesc_can_actuators

#include <stdio.h>
#include <stdlib.h>
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

LOG_MODULE_REGISTER(actuate_vesc_can, CONFIG_CEREBRI_ACTUATE_VESC_CAN_LOG_LEVEL);

#define CONFIG_VESC_CAN_ACTUATORS_INIT_PRIORITY 90
#define MY_STACK_SIZE                      4096
#define MY_PRIORITY                        4

extern struct perf_duration control_latency;

typedef enum vesc_can_type_t {
	VESC_CAN_TYPE_NORMALIZED = 0,
	VESC_CAN_TYPE_POSITION = 1,
	VESC_CAN_TYPE_VELOCITY = 2,
} vesc_can_type_t;

typedef struct actuator_vesc_can_t {
	const char *label;
	bool fd;
	uint8_t id;
	uint8_t index;
	uint8_t pole_pair;
	uint8_t bus_id;
	const struct device * device;
	bool disarmed;
	vesc_can_type_t type;
} actuator_vesc_can_t;

struct context {
	synapse_pb_Actuators actuators;
	synapse_pb_Status status;
	struct zros_node node;
	struct zros_sub sub_actuators, sub_status;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	uint8_t num_actuators;
	const actuator_vesc_can_t *actuator_vesc_cans;
};

static int actuate_vesc_can_init(struct context *ctx)
{
	LOG_INF("init");
	zros_node_init(&ctx->node, "actuate_vesc_can");
	zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1000);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	k_sem_take(&ctx->running, K_FOREVER);
	return 0;
}

static void actuate_vesc_can_fini(struct context *ctx)
{
	LOG_INF("fini");
	zros_sub_fini(&ctx->sub_actuators);
	zros_sub_fini(&ctx->sub_status);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
}

static void vesc_can_update(struct context *ctx)
{
	bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
	int err = 0;

	for (int i = 0; i < ctx->num_actuators; i++) {
		actuator_vesc_can_t vesc_can = ctx->actuator_vesc_cans[i];

		uint32_t pulse = vesc_can.disarmed;
		double input = 0;

		if (vesc_can.type == VESC_CAN_TYPE_NORMALIZED) {
			input = ctx->actuators.normalized[vesc_can.index];
		} else if (vesc_can.type == VESC_CAN_TYPE_POSITION) {
			input = ctx->actuators.position[vesc_can.index];
		} else if (vesc_can.type == VESC_CAN_TYPE_VELOCITY) {
			input = ctx->actuators.velocity[vesc_can.index];
		}

		if (armed) {
			//pulse = (uint32_t)((vesc_can.scale * input) + vesc_can.center);
		}

		//err = vesc_can_set_pulse_dt(&vesc_can.device, VESC_CAN_NSEC(pulse));
		perf_duration_stop(&control_latency);

		if (err) {
			LOG_ERR("Failed to set pulse %d on %d (err %d)", pulse, vesc_can.index, err);
		}
	}
}

static void actuate_vesc_can_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = actuate_vesc_can_init(ctx);

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

		// update vesc_can
		vesc_can_update(ctx);
	}

	actuate_vesc_can_fini(ctx);
}

static int start(struct context *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);
	k_tid_t tid = k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				      actuate_vesc_can_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "actuate_vesc_can");
	k_thread_start(tid);
	return 0;
}

static int actuate_vesc_can_cmd_handler(const struct shell *sh, size_t argc, char **argv, void *data)
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
	} else {
		shell_print(sh, "unknown command");
	}
	return 0;
}

static int actuate_vesc_can_device_init(const struct device *dev)
{
	struct context *data = dev->data;
	start(data);
	return 0;
}

#define VESC_CAN_ACTUATOR_SHELL(inst)                                                              \
	SHELL_SUBCMD_DICT_SET_CREATE(sub_actuate_vesc_can_##inst, actuate_vesc_can_cmd_handler,    \
				     (start, &data_##inst, "start"), (stop, &data_##inst, "stop"), \
				     (status, &data_##inst, "status"));                            \
	SHELL_CMD_REGISTER(actuate_vesc_can_##inst, &sub_actuate_vesc_can_##inst, "actuate_vesc_can commands", \
			   NULL);

#define VESC_CAN_ACTUATOR_DEFINE(node_id)                                                          \
	{                                                                                          \
		.label = DT_NODE_FULL_NAME(node_id),                                               \
		.fd = DT_PROP(node_id, fd),                                                        \
		.index = DT_PROP(node_id, input_index),                                            \
		.pole_pair = DT_PROP(node_id, pole_pair),                                          \
		.bus_id = DT_PROP(node_id, bus_id),                                                \
		.device = DEVICE_DT_GET(DT_PROP(node_id, device)),                                 \
		.type = DT_ENUM_IDX(node_id, input_type),                                          \
		.disarmed = true,                                                                  \
	},

#define VESC_CAN_ACTUATORS_DEFINE(inst)                                                            \
	static const actuator_vesc_can_t g_actuator_vesc_cans_##inst[] = {                         \
		DT_FOREACH_CHILD(                                                                  \
			DT_INST(inst, cerebri_vesc_can_actuators), VESC_CAN_ACTUATOR_DEFINE)};     \
	static K_THREAD_STACK_DEFINE(g_my_stack_area_##inst, MY_STACK_SIZE);                       \
	static struct context data_##inst = {                                                      \
		.actuators = synapse_pb_Actuators_init_default,                                    \
		.status = synapse_pb_Status_init_default,                                          \
		.node = {},                                                                        \
		.sub_status = {},                                                                  \
		.sub_actuators = {},                                                               \
		.running = Z_SEM_INITIALIZER(data_##inst.running, 1, 1),                           \
		.stack_size = MY_STACK_SIZE,                                                       \
		.stack_area = g_my_stack_area_##inst,                                              \
		.thread_data = {},                                                                 \
		.actuator_vesc_cans = g_actuator_vesc_cans_##inst,                                 \
		.num_actuators = DT_CHILD_NUM(DT_INST(inst, cerebri_vesc_can_actuators)),          \
	};                                                                                         \
	VESC_CAN_ACTUATOR_SHELL(inst);                                                             \
	DEVICE_DT_INST_DEFINE(inst, actuate_vesc_can_device_init, NULL, &data_##inst, NULL,        \
			      POST_KERNEL, CONFIG_VESC_CAN_ACTUATORS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(VESC_CAN_ACTUATORS_DEFINE)
