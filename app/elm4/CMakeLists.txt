#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

if (NOT DEFINED BOARD)
  set(BOARD "native_posix_64")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(elm4 LANGUAGES C VERSION 0.0.1)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)

configure_file(app_version.h.in ${CMAKE_BINARY_DIR}/app/include/app_version.h)
target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/app/include src)

set(SOURCE_FILES
  src/main.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})
