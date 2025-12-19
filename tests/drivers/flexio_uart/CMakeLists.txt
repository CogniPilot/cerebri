# Copyright 2024 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(flexio_uart_test)

target_sources(app PRIVATE src/main.c)

# Include the FlexIO UART driver header
target_include_directories(app PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/zephyr/misc/nxp_flexio_uart
)
