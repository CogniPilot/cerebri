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

project(pubsub LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Wno-unused-parameter -Werror)

set(SOURCE_FILES
  src/main.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})
