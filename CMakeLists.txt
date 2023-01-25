cmake_minimum_required(VERSION 3.20.0)

enable_language(CXX)

set(BOARD native_posix_64)

set(ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/zenoh-pico
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sim_gz
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sim_core
    )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(cerebri)

add_subdirectory(apps/rover)