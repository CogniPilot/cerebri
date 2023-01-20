# SPDX-License-Identifier: Apache-2.0
set(BOARD native_posix)

set(ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../modules/zenoh-pico
    )
    
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tcp)

target_compile_definitions(app -fexceptions)

target_sources(app PRIVATE src/main.c)
set_property(TARGET app PROPERTY CXX_STANDARD 17)