#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

if (${BOARD} STREQUAL "mr_canhubk3")
  message(STATUS "enabling mr_canhubk3_adap shield")
  set(SHIELD mr_canhubk3_adap)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(b3rb LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)

set_source_files_properties(src/casadi/gen/b3rb.c PROPERTIES COMPILE_FLAGS -Wno-unused-parameter)

set(SOURCE_FILES src/main.c)

if (CONFIG_CEREBRI_B3RB_FSM)
  list(APPEND SOURCE_FILES src/fsm.c)
endif()

if (CONFIG_CEREBRI_B3RB_ESTIMATE)
  list(APPEND SOURCE_FILES src/estimate.c)
endif()

if (CONFIG_CEREBRI_B3RB_MIXING)
  list(APPEND SOURCE_FILES src/mixing.c)
endif()

if (CONFIG_CEREBRI_B3RB_LIGHTING)
  list(APPEND SOURCE_FILES src/lighting.c)
endif()

if (CONFIG_CEREBRI_B3RB_MANUAL)
  list(APPEND SOURCE_FILES src/manual.c)
endif()

if (CONFIG_CEREBRI_B3RB_POSITION)
  list(APPEND SOURCE_FILES
    src/position.c)
endif()

if (CONFIG_CEREBRI_B3RB_VELOCITY)
  list(APPEND SOURCE_FILES
    src/velocity.c)
endif()

if (CONFIG_CEREBRI_B3RB_CASADI)
  list(APPEND SOURCE_FILES
    src/casadi/gen/b3rb.c)
endif()

target_sources(app PRIVATE ${SOURCE_FILES})
