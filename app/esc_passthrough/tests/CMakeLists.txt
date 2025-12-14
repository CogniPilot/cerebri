# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 CogniPilot Foundation

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esc_passthrough_tests)

# Protocol tests (run in CI - no hardware needed)
target_sources(app PRIVATE
    test_protocol.c
)

# Hardware tests (only on real boards with ESC GPIOs)
if(CONFIG_GPIO)
    target_sources_ifdef(CONFIG_TEST_HARDWARE app PRIVATE
        test_hardware.c
    )
endif()
