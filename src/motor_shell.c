/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_output.h"

#include "topic_shell.h"

#include <errno.h>
#include <stdlib.h>

#include <zephyr/drivers/misc/nxp_flexio_dshot/nxp_flexio_dshot.h>
#include <zephyr/shell/shell.h>

static int cmd_motor_status(const struct shell *sh, size_t argc, char **argv)
{
	cerebri_topic_MotorValues4f_t test_motors = {0};
	cerebri_topic_MotorRaw4u16_t test_raw = {0};
	cerebri_topic_MotorValues4f_t motors = {0};
	cerebri_topic_MotorRaw4u16_t raw = {0};
	bool active;
	bool raw_active;
	bool armed;
	bool test_mode;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	active = cerebri_motor_test_get(&test_motors);
	raw_active = cerebri_motor_raw_test_get(&test_raw);
	(void)cerebri_topic_motor_output_get(&motors, &raw, &armed, &test_mode);
	shell_print(sh,
		    "motor_test=%d test_m0=%0.3f test_m1=%0.3f test_m2=%0.3f test_m3=%0.3f",
		    active ? 1 : 0,
		    (double)test_motors.m0,
		    (double)test_motors.m1,
		    (double)test_motors.m2,
		    (double)test_motors.m3);
	shell_print(sh,
		    "raw_test=%d raw_m0=%u raw_m1=%u raw_m2=%u raw_m3=%u",
		    raw_active ? 1 : 0,
		    (unsigned int)test_raw.m0,
		    (unsigned int)test_raw.m1,
		    (unsigned int)test_raw.m2,
		    (unsigned int)test_raw.m3);
	shell_print(sh,
		    "last_out armed=%d test_mode=%d m0=%0.3f/%u m1=%0.3f/%u m2=%0.3f/%u m3=%0.3f/%u",
		    armed ? 1 : 0,
		    test_mode ? 1 : 0,
		    (double)motors.m0, (unsigned int)raw.m0,
		    (double)motors.m1, (unsigned int)raw.m1,
		    (double)motors.m2, (unsigned int)raw.m2,
		    (double)motors.m3, (unsigned int)raw.m3);
	shell_print(sh, "guess: m0=front-right m1=rear-right m2=rear-left m3=front-left");

	return 0;
}

static int cmd_motor_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	cerebri_motor_test_clear();
	cerebri_motor_raw_test_clear();
	if (cerebri_motor_output_ready()) {
		cerebri_motor_output_write_all(&(cerebri_topic_MotorValues4f_t){0}, false, false);
	}
	shell_print(sh, "all motor tests stopped");

	return 0;
}

static int cmd_motor_spin(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long index;
	float value;

	ARG_UNUSED(argc);

	if (!cerebri_motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	index = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || index < 0 || index > 3) {
		shell_error(sh, "index must be 0..3");
		return -EINVAL;
	}

	end = NULL;
	value = strtof(argv[2], &end);
	if (*argv[2] == '\0' || *end != '\0' || value < 0.0f || value > 1.0f) {
		shell_error(sh, "value must be 0.0..1.0");
		return -EINVAL;
	}

	cerebri_motor_test_set((size_t)index, value);
	shell_print(sh, "motor %ld set to %0.3f", index, (double)value);

	return 0;
}

static int cmd_motor_raw(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long index;
	long value;

	ARG_UNUSED(argc);

	if (!cerebri_motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	index = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || index < 0 || index > 3) {
		shell_error(sh, "index must be 0..3");
		return -EINVAL;
	}

	end = NULL;
	value = strtol(argv[2], &end, 10);
	if (*argv[2] == '\0' || *end != '\0' || value < 0 || value > DSHOT_MAX) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}
	if (value != 0 && value < DSHOT_MIN) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}

	cerebri_motor_test_clear();
	cerebri_motor_raw_test_set((size_t)index, (uint16_t)value);
	shell_print(sh, "raw motor %ld set to %ld", index, value);

	return 0;
}

static int cmd_motor_raw_all(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	long value;

	ARG_UNUSED(argc);

	if (!cerebri_motor_output_ready()) {
		shell_error(sh, "dshot not ready");
		return -ENODEV;
	}

	value = strtol(argv[1], &end, 10);
	if (*argv[1] == '\0' || *end != '\0' || value < 0 || value > DSHOT_MAX) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}
	if (value != 0 && value < DSHOT_MIN) {
		shell_error(sh, "value must be 0 or 48..2047");
		return -EINVAL;
	}

	cerebri_motor_test_clear();
	cerebri_motor_raw_test_set_all((uint16_t)value);
	shell_print(sh, "all raw motors set to %ld", value);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_motor,
	SHELL_CMD_ARG(spin, NULL, "spin one motor: motor spin <index:0..3> <value:0..1>",
		      cmd_motor_spin, 3, 0),
	SHELL_CMD_ARG(raw, NULL, "set one motor raw: motor raw <index:0..3> <value:0|48..2047>",
		      cmd_motor_raw, 3, 0),
	SHELL_CMD_ARG(raw_all, NULL, "set all motors raw: motor raw_all <value:0|48..2047>",
		      cmd_motor_raw_all, 2, 0),
	SHELL_CMD(stop, NULL, "stop all motor tests", cmd_motor_stop),
	SHELL_CMD(status, NULL, "show motor test state", cmd_motor_status),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(motor, &sub_motor, "bench motor commands", NULL);
