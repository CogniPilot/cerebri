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

#include <zephyr/net/socket.h>
#include <zephyr/net/socketcan.h>
#include <zephyr/net/socketcan_utils.h>

#include <cerebri/core/perf_duration.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(actuate_vesc_can, CONFIG_CEREBRI_ACTUATE_VESC_CAN_LOG_LEVEL);

#define CONFIG_VESC_CAN_ACTUATORS_INIT_PRIORITY 90
#define MY_STACK_SIZE                           4096
#define MY_PRIORITY                             4

#define CAN_STATUS_ID_BASE 0x00000900

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint32_t g_send_count = 0;

extern struct perf_duration control_latency;

static void actuate_vesc_can_rx_callback(const struct device *dev, struct can_frame *frame,
					 void *user_data);

typedef enum vesc_can_type_t {
	VESC_CAN_TYPE_NORMALIZED = 0,
	VESC_CAN_TYPE_POSITION = 1,
	VESC_CAN_TYPE_VELOCITY = 2,
} vesc_can_type_t;

struct actuator_vesc_can {
	const char *label;
	uint8_t id;
	uint8_t index;
	uint8_t pole_pair;
	uint8_t vesc_id;
	vesc_can_type_t type;
	struct can_filter rx_filter;
	struct context *ctx;
	double rotation;
};

struct context {
	synapse_pb_Actuators actuators;
	synapse_pb_WheelOdometry wheel_odometry;
	synapse_pb_Status status;
	struct zros_node node;
	struct zros_sub sub_actuators, sub_status;
	struct zros_pub pub_wheel_odometry;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	uint8_t num_actuators;
	struct actuator_vesc_can *actuator_vesc_cans;
	const struct device *device;
	bool ready;
	bool fd;
	const char *label;
	uint16_t status_rate;
	bool enable_pub_wheel_odom;
};

static int actuate_vesc_can_stop(struct context *ctx)
{
	int err = 0;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	ctx->ready = false;
	err = can_get_state(ctx->device, &state, &err_cnt);
	if (err != 0) {
		LOG_ERR("%s - failed to get CAN controller state (%d)\n", ctx->label, err);
		ctx->ready = false;
		return err;
	}

	// make sure stopped
	if (state != CAN_STATE_STOPPED) {
		err = can_stop(ctx->device);
		if (err != 0) {
			LOG_ERR("%s - failed to stop (%d)\n", ctx->label, err);
			return err;
		}
	}

	return err;
}

static int actuate_vesc_can_init(struct context *ctx)
{
	int err = 0;
	zros_node_init(&ctx->node, "actuate_vesc_can");
	zros_sub_init(&ctx->sub_actuators, &ctx->node, &topic_actuators, &ctx->actuators, 1000);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 10);
	zros_pub_init(&ctx->pub_wheel_odometry, &ctx->node, &topic_wheel_odometry,
		      &ctx->wheel_odometry);

	if (ctx->ready) {
		LOG_ERR("%s - already initialized", ctx->label);
		return 0;
	}

	// check if device ready
	if (!device_is_ready(ctx->device)) {
		LOG_ERR("%s - device not ready\n", ctx->label);
		ctx->ready = false;
		return -1;
	}

	actuate_vesc_can_stop(ctx);

	// set device mode
	if (ctx->fd) {
		err = can_set_mode(ctx->device, CAN_MODE_FD);
		if (err != 0) {
			LOG_ERR("%s - set mode FD failed (%d)\n", ctx->label, err);
			return err;
		}
	}

	// start device
	err = can_start(ctx->device);
	if (err != 0) {
		LOG_ERR("%s - start failed\n", ctx->label);
		return err;
	}

	// add receive callback
	for (int i = 0; i < ctx->num_actuators; i++) {
		struct actuator_vesc_can *act = &ctx->actuator_vesc_cans[i];
		act->ctx = ctx;
		act->rx_filter.flags = CAN_FILTER_IDE;
		act->rx_filter.id = CAN_STATUS_ID_BASE + act->vesc_id;
		act->rx_filter.mask = CAN_EXT_ID_MASK;
		err = can_add_rx_filter(ctx->device, actuate_vesc_can_rx_callback, act,
					&act->rx_filter);
		if (err < 0) {
			LOG_ERR("%s - callback setup failed (%d)\n", ctx->label, err);
			return err;
		}
	}

	ctx->ready = true;
	LOG_DBG("%s - connected and properly initialized.\n", ctx->label);
	k_sem_take(&ctx->running, K_FOREVER);
	LOG_INF("init");
	return 0;
}

static int actuate_vesc_can_fini(struct context *ctx)
{
	actuate_vesc_can_stop(ctx);
	zros_sub_fini(&ctx->sub_actuators);
	zros_sub_fini(&ctx->sub_status);
	zros_pub_fini(&ctx->pub_wheel_odometry);
	zros_node_fini(&ctx->node);
	k_sem_give(&ctx->running);
	LOG_INF("fini");
	return 0;
}

static void actuate_vesc_can_rx_callback(const struct device *dev, struct can_frame *frame,
					 void *user_data)
{
	struct actuator_vesc_can *act = (struct actuator_vesc_can *)user_data;
	struct context *ctx = act->ctx;
	int64_t data = (frame->data[0] << 24) + (frame->data[1] << 16) + (frame->data[2] << 8) +
		       frame->data[3];
	act->rotation += 2 * M_PI * data / (act->pole_pair * ctx->status_rate * 60);
	// LOG_INF("Data: %ld", data);
	// LOG_INF("Angular: %f", act->rotation);
	if (act->id == ctx->actuator_vesc_cans[0].id) {
		double mean_rotation = 0;
		for (int i = 0; i < ctx->num_actuators; i++) {
			mean_rotation += ctx->actuator_vesc_cans[i].rotation;
		}
		mean_rotation /= ctx->num_actuators;
		ctx->wheel_odometry.has_stamp = true;
		stamp_msg(&ctx->wheel_odometry.stamp, k_uptime_ticks());
		ctx->wheel_odometry.rotation = mean_rotation;
	}
}

static void actuate_vesc_can_update(struct context *ctx)
{
	bool armed = ctx->status.arming == synapse_pb_Status_Arming_ARMING_ARMED;
	int err = 0;

	for (int i = 0; i < ctx->num_actuators; i++) {
		struct actuator_vesc_can *act = &ctx->actuator_vesc_cans[i];

		struct can_frame frame = {
			.dlc = can_bytes_to_dlc(4),
			.flags = CAN_FRAME_IDE,
		};
		double input = 0;

		if (act->type == VESC_CAN_TYPE_VELOCITY && armed) {
			input = ctx->actuators.velocity[act->index];
		}

		int32_t erpm = act->pole_pair * input * 60 / (2 * M_PI);
		frame.id = 768 + act->vesc_id;
		frame.data[0] = erpm >> 24 & 255;
		frame.data[1] = erpm >> 16 & 255;
		frame.data[2] = erpm >> 8 & 255;
		frame.data[3] = erpm & 255;

		// send can data
		g_send_count += 1;
		err = can_send(ctx->device, &frame, K_NO_WAIT, NULL, NULL);
		if (err != 0) {
			ctx->ready = false;
			LOG_ERR("%s - send failed to VESC ID: %d (%d)\n", act->label, act->vesc_id,
				err);
			continue;
		}
		perf_duration_stop(&control_latency);
	}
}

static void actuate_vesc_can_run(void *p0, void *p1, void *p2)
{
	struct context *ctx = p0;
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	int ret = actuate_vesc_can_init(ctx);

	if (ret < 0) {
		actuate_vesc_can_fini(ctx);
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
				LOG_ERR("Disarming motors due to actuator msg timeout!");
			}
		}

		if (zros_sub_update_available(&ctx->sub_status)) {
			zros_sub_update(&ctx->sub_status);
		}

		if (zros_sub_update_available(&ctx->sub_actuators)) {
			zros_sub_update(&ctx->sub_actuators);
		}

		zros_pub_update(&ctx->pub_wheel_odometry);

		// update vesc_can
		actuate_vesc_can_update(ctx);
	}

	actuate_vesc_can_fini(ctx);
}

static int start(struct context *ctx)
{
	__ASSERT_NO_MSG(ctx != NULL);
	k_tid_t tid =
		k_thread_create(&ctx->thread_data, ctx->stack_area, ctx->stack_size,
				actuate_vesc_can_run, ctx, NULL, NULL, MY_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(tid, "actuate_vesc_can");
	k_thread_start(tid);
	return 0;
}

static int actuate_vesc_can_cmd_handler(const struct shell *sh, size_t argc, char **argv,
					void *data)
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
	SHELL_CMD_REGISTER(actuate_vesc_can_##inst, &sub_actuate_vesc_can_##inst,                  \
			   "actuate_vesc_can commands", NULL);

#define VESC_CAN_ACTUATOR_DEFINE(node_id)                                                          \
	{                                                                                          \
		.label = DT_NODE_FULL_NAME(node_id),                                               \
		.index = DT_PROP(node_id, input_index),                                            \
		.pole_pair = DT_PROP(node_id, pole_pair),                                          \
		.vesc_id = DT_PROP(node_id, vesc_id),                                              \
		.type = DT_ENUM_IDX(node_id, input_type),                                          \
		.rx_filter = {},                                                                   \
		.ctx = NULL,                                                                       \
		.rotation = 0,                                                                     \
	},

#define VESC_CAN_ACTUATORS_DEFINE(inst)                                                            \
	static struct actuator_vesc_can g_actuator_vesc_cans_##inst[] = {                          \
		DT_FOREACH_CHILD(DT_DRV_INST(inst), VESC_CAN_ACTUATOR_DEFINE)};                    \
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
		.num_actuators = DT_CHILD_NUM(DT_DRV_INST(inst)),                                  \
		.device = DEVICE_DT_GET(DT_INST_PROP(inst, device)),                               \
		.ready = false,                                                                    \
		.fd = DT_INST_PROP(inst, fd),                                                      \
		.status_rate = DT_INST_PROP(inst, status_rate),                                    \
		.enable_pub_wheel_odom = DT_INST_PROP(inst, pub_wheel_odometry),                   \
		.label = DT_NODE_FULL_NAME(DT_DRV_INST(inst)),                                     \
	};                                                                                         \
	VESC_CAN_ACTUATOR_SHELL(inst);                                                             \
	DEVICE_DT_INST_DEFINE(inst, actuate_vesc_can_device_init, NULL, &data_##inst, NULL,        \
			      POST_KERNEL, CONFIG_VESC_CAN_ACTUATORS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(VESC_CAN_ACTUATORS_DEFINE)
