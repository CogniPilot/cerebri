# Copyright 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_NXP_FLEXIO_UART nxp_flexio_uart.c)
zephyr_include_directories(.)
