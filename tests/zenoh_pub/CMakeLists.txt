# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ZENOH_PICO)

cmake_minimum_required(VERSION 3.20.0)


#set(CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake")
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

#find_package(ignition-transport11 REQUIRED)

set(ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/zenoh-pico
    )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zenoh_pub)

target_sources(app PRIVATE src/main.c)
target_compile_definitions(app PRIVATE ZENOH_ZEPHYR)

include(${ZEPHYR_BASE}/samples/net/common/common.cmake)

endif(CONFIG_ZENOH_PICO)