#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(elm4 LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)

set_source_files_properties(src/casadi/gen/elm4.c PROPERTIES COMPILE_FLAGS -Wno-unused-parameter)

set(SOURCE_FILES
  src/estimate.c
  src/fsm.c
  src/lighting.c
  src/main.c
  src/manual.c
  src/mixing.c
  src/position.c
  src/velocity.c
  src/casadi/gen/elm4.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})
