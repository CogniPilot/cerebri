/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_output.h"

#include "topic_shell.h"

#include <zephyr/device.h>
#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#define DSHOT_NODE DT_ALIAS(motors)

static atomic_t g_motor_test_active;
static cerebri_topic_MotorValues4f_t g_motor_test_values;
static atomic_t g_motor_raw_test_active;
static cerebri_topic_MotorRaw4u16_t g_motor_raw_test_values;

static float clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static uint16_t motor_to_dshot(float normalized, bool armed)
{
	float min_output = armed ? CEREBRI_MOTOR_IDLE_THROTTLE : 0.0f;
	float clamped = clampf(normalized, min_output, 1.0f);
	float span = (float)(DSHOT_MAX - DSHOT_MIN);

	if (!armed || clamped <= 0.0f) {
		return DSHOT_DISARMED;
	}

	return (uint16_t)(DSHOT_MIN + (clamped * span) + 0.5f);
}

void cerebri_motor_output_init(void)
{
	cerebri_motor_test_clear();
	cerebri_motor_raw_test_clear();
	cerebri_topic_motor_output_publish(&(cerebri_topic_MotorValues4f_t){0},
					    &(cerebri_topic_MotorRaw4u16_t){
						    .m0 = DSHOT_DISARMED,
						    .m1 = DSHOT_DISARMED,
						    .m2 = DSHOT_DISARMED,
						    .m3 = DSHOT_DISARMED,
					    },
					    false, false);
}

bool cerebri_motor_output_ready(void)
{
	return device_is_ready(DEVICE_DT_GET(DSHOT_NODE));
}

void cerebri_motor_output_write_all(const cerebri_topic_MotorValues4f_t *motors, bool armed,
				     bool test_mode)
{
	float min_output = (armed && !test_mode) ? CEREBRI_MOTOR_IDLE_THROTTLE : 0.0f;
	cerebri_topic_MotorValues4f_t applied = {0};
	cerebri_topic_MotorRaw4u16_t raw = {0};
	const float *motor_values = cerebri_topic_motor_values_data_const(motors);
	float *applied_values = cerebri_topic_motor_values_data(&applied);
	uint16_t *raw_values = cerebri_topic_motor_raw_data(&raw);

	for (size_t i = 0; i < 4U; i++) {
		applied_values[i] = armed ? clampf(motor_values[i], min_output, 1.0f) : 0.0f;
		raw_values[i] = motor_to_dshot(applied_values[i], armed && applied_values[i] > 0.0f);
		nxp_flexio_dshot_data_set(DEVICE_DT_GET(DSHOT_NODE), i, raw_values[i], false);
	}

	cerebri_topic_motor_output_publish(&applied, &raw, armed, test_mode);
	nxp_flexio_dshot_trigger(DEVICE_DT_GET(DSHOT_NODE));
}

void cerebri_motor_output_write_all_raw(const cerebri_topic_MotorRaw4u16_t *raw, bool test_mode)
{
	cerebri_topic_MotorValues4f_t applied = {0};
	cerebri_topic_MotorRaw4u16_t clamped = {0};
	const uint16_t *raw_values = cerebri_topic_motor_raw_data_const(raw);
	float *applied_values = cerebri_topic_motor_values_data(&applied);
	uint16_t *clamped_values = cerebri_topic_motor_raw_data(&clamped);
	bool armed = false;

	for (size_t i = 0; i < 4U; i++) {
		uint16_t value = raw_values[i];

		if (value != DSHOT_DISARMED && value < DSHOT_MIN) {
			value = DSHOT_MIN;
		}
		if (value > DSHOT_MAX) {
			value = DSHOT_MAX;
		}

		clamped_values[i] = value;
		if (value == DSHOT_DISARMED) {
			applied_values[i] = 0.0f;
		} else {
			applied_values[i] = (float)(value - DSHOT_MIN) / (float)(DSHOT_MAX - DSHOT_MIN);
			armed = true;
		}
		nxp_flexio_dshot_data_set(DEVICE_DT_GET(DSHOT_NODE), i, value, false);
	}

	cerebri_topic_motor_output_publish(&applied, &clamped, armed, test_mode);
	nxp_flexio_dshot_trigger(DEVICE_DT_GET(DSHOT_NODE));
}

bool cerebri_motor_test_get(cerebri_topic_MotorValues4f_t *motors)
{
	const float *test_values = cerebri_topic_motor_values_data_const(&g_motor_test_values);
	float *motor_values = cerebri_topic_motor_values_data(motors);
	bool active;
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4U; i++) {
		motor_values[i] = test_values[i];
	}
	active = atomic_get(&g_motor_test_active) != 0;

	irq_unlock(key);

	return active;
}

void cerebri_motor_test_set(size_t index, float value)
{
	float *test_values = cerebri_topic_motor_values_data(&g_motor_test_values);
	unsigned int key = irq_lock();

	test_values[index] = clampf(value, 0.0f, 1.0f);
	atomic_set(&g_motor_test_active, 1);

	irq_unlock(key);
}

void cerebri_motor_test_clear(void)
{
	float *test_values = cerebri_topic_motor_values_data(&g_motor_test_values);
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4U; i++) {
		test_values[i] = 0.0f;
	}
	atomic_set(&g_motor_test_active, 0);

	irq_unlock(key);
}

bool cerebri_motor_raw_test_get(cerebri_topic_MotorRaw4u16_t *raw)
{
	const uint16_t *test_values = cerebri_topic_motor_raw_data_const(&g_motor_raw_test_values);
	uint16_t *raw_values = cerebri_topic_motor_raw_data(raw);
	bool active;
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4U; i++) {
		raw_values[i] = test_values[i];
	}
	active = atomic_get(&g_motor_raw_test_active) != 0;

	irq_unlock(key);

	return active;
}

void cerebri_motor_raw_test_set(size_t index, uint16_t value)
{
	uint16_t *test_values = cerebri_topic_motor_raw_data(&g_motor_raw_test_values);
	unsigned int key = irq_lock();

	test_values[index] = value;
	atomic_set(&g_motor_raw_test_active, 1);

	irq_unlock(key);
}

void cerebri_motor_raw_test_set_all(uint16_t value)
{
	uint16_t *test_values = cerebri_topic_motor_raw_data(&g_motor_raw_test_values);
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4U; i++) {
		test_values[i] = value;
	}
	atomic_set(&g_motor_raw_test_active, 1);

	irq_unlock(key);
}

void cerebri_motor_raw_test_clear(void)
{
	uint16_t *test_values = cerebri_topic_motor_raw_data(&g_motor_raw_test_values);
	unsigned int key = irq_lock();

	for (size_t i = 0; i < 4U; i++) {
		test_values[i] = DSHOT_DISARMED;
	}
	atomic_set(&g_motor_raw_test_active, 0);

	irq_unlock(key);
}
