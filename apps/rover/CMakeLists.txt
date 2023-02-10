cmake_minimum_required(VERSION 3.20.0)

enable_language(CXX)

set(BOARD native_posix_64)

#set(ZEPHYR_EXTRA_MODULES
#    ${CMAKE_CURRENT_SOURCE_DIR}/../
#    )
#    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/zenoh-pico
#    )
#
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(cerebri)


target_sources(app PRIVATE
    src/main.c
    src/estimator.c
    src/mixer.c
    src/controller.c
    src/casadi/rover.c
)
