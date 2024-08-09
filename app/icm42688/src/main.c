/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>


int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	struct spi_dt_spec spec = SPI_DT_SPEC_GET(DT_NODELABEL(slow), SPI_WORD_SET(8) | SPI_MODE_GET(0), 1);
	(void)spec;

	/*
	struct spi_dt_spec spec = {
		.bus
	};
	spi_transceive_dt(
	*/

	return 0;
}
