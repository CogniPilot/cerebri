#-------------------------------------------------------------------------------
# Zephyr Cerebri ESC Passthrough
#
# Copyright (c) 2024 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0
#
# ESC passthrough for programming ESCs using BLHeli Suite, BLHeli Configurator,
# ESC Configurator, or similar tools. Implements the 4-way interface protocol.

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(esc_passthrough LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Wno-unused-parameter -Werror)

set(SOURCE_FILES
  src/main.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})
