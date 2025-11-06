/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <synapse_topic_list.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

LOG_MODULE_REGISTER(sense_power, CONFIG_CEREBRI_SENSE_POWER_LOG_LEVEL);

#define MY_STACK_SIZE 2048
#define MY_PRIORITY   6

extern struct k_work_q g_low_priority_work_q;

#define N_SENSORS 1

void power_work_handler(struct k_work *work);

typedef struct context {
	struct k_work work_item;
	const struct device *device[N_SENSORS];
	struct zros_node node;
	struct zros_pub pub;
	synapse_pb_BatteryState data;
} context_t;

static context_t g_ctx = {
	.work_item = Z_WORK_INITIALIZER(power_work_handler),
	.device = {},
	.node = {},
	.pub = {},
	.data = {
		.has_stamp = true,
		.stamp = synapse_pb_Timestamp_init_default,
		.capacity = 0,
		.cell_temperature = {},
		.cell_temperature_count = 0,
		.cell_voltage = {},
		.cell_voltage_count = 0,
		.charge = 0,
		.current = 0,
		.design_capacity = 0,
		.location = "",
		.percentage = 0,
		.power_supply_health = synapse_pb_BatteryState_PowerSupplyHealth_UNKNOWN_HEALTH,
		.power_supply_technology =
			synapse_pb_BatteryState_PowerSupplyTechnology_UNKNOWN_TECHNOLOGY,
		.power_supply_status = synapse_pb_BatteryState_PowerSupplyStatus_UNKNOWN_STATUS,
		.present = true,
		.serial_number = "0",
		.temperature = 0,
		.voltage = 0,
	}};

void power_work_handler(struct k_work *work)
{
	context_t *ctx = CONTAINER_OF(work, context_t, work_item);
	int ret = sensor_sample_fetch(ctx->device[0]);
	if (ret) {
		LOG_ERR_RATELIMIT_RATE(2000, "Could not fetch sensor data");
		return;
	}
	struct sensor_value voltage, current;

	sensor_channel_get(ctx->device[0], SENSOR_CHAN_VOLTAGE, &voltage);
	sensor_channel_get(ctx->device[0], SENSOR_CHAN_CURRENT, &current);

	stamp_msg(&ctx->data.stamp, k_uptime_ticks());
	ctx->data.voltage = sensor_value_to_double(&voltage);
	ctx->data.current = sensor_value_to_double(&current);

	zros_pub_update(&ctx->pub);
}

void power_timer_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&g_low_priority_work_q, &g_ctx.work_item);
}

K_TIMER_DEFINE(power_timer, power_timer_handler, NULL);

int sense_power_entry_point(context_t *ctx)
{
	LOG_INF("init");
	ctx->device[0] = DEVICE_DT_GET(DT_ALIAS(power0));
	if (!device_is_ready(ctx->device[0])) {
		LOG_ERR("Device %s is not ready", ctx->device[0]->name);
	}
	zros_node_init(&ctx->node, "sense_power");
	zros_pub_init(&ctx->pub, &ctx->node, &topic_battery_state, &ctx->data);
	k_timer_start(&power_timer, K_MSEC(100), K_MSEC(100));
	return 0;
}

K_THREAD_DEFINE(sense_power, MY_STACK_SIZE, sense_power_entry_point, &g_ctx, NULL, NULL,
		MY_PRIORITY, 0, 100);

/* vi: ts=4 sw=4 et */
