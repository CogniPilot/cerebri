# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/../modules/tinyframe
  ${CMAKE_CURRENT_SOURCE_DIR}/../modules/synapse)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/../modules)


set(ENV{PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION} python)
message(STATUS "env: $ENV{PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(protobuf_ethernet)

target_sources(app PRIVATE
  src/socket_echo.c
)
