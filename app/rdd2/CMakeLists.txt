#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(rdd2 LANGUAGES C)

set(CYECCA_PYTHON  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/cyecca_python)

set(flags
  -std=c11
  -Wall
  -Wextra
  -Werror
  -Wstrict-prototypes
  -Waggregate-return
  -Wbad-function-cast
  -Wcast-align
  -Wcast-qual
  -Wfloat-equal
  -Wformat-security
  -Wlogical-op
  -Wmissing-declarations
  # ubxlib includes geographiclib which dir doesn't exist
  #-Wmissing-include-dirs
  -Wmissing-prototypes
  -Wnested-externs
  -Wpointer-arith
  -Wredundant-decls
  -Wsequence-point
  -Wshadow
  -Wstrict-prototypes
  -Wswitch
  -Wundef
  -Wunreachable-code
  -Wunused-but-set-parameter
  -Wwrite-strings
  )
string(JOIN " " flags ${flags})

set(SOURCE_FILES
  src/boot_banner.c
  src/input_mapping.c
  )

if (CONFIG_CEREBRI_RDD2_FSM)
  list(APPEND SOURCE_FILES src/fsm.c)
endif()

if (CONFIG_CEREBRI_RDD2_ESTIMATE)
  list(APPEND SOURCE_FILES src/estimate.c)
endif()

if (CONFIG_CEREBRI_RDD2_ALLOCATION)
  list(APPEND SOURCE_FILES src/allocation.c)
endif()

if (CONFIG_CEREBRI_RDD2_LIGHTING)
  list(APPEND SOURCE_FILES src/lighting.c)
endif()

if (CONFIG_CEREBRI_RDD2_COMMAND)
  list(APPEND SOURCE_FILES src/command.c)
endif()

if (CONFIG_CEREBRI_RDD2_POSITION)
  list(APPEND SOURCE_FILES
    src/position.c)
endif()

if (CONFIG_CEREBRI_RDD2_ANGULAR_VELOCITY)
  list(APPEND SOURCE_FILES
    src/angular_velocity.c)
endif()

if (CONFIG_CEREBRI_RDD2_ATTITUDE)
  list(APPEND SOURCE_FILES
    src/attitude.c)
endif()

set(CASADI_DEST_DIR ${CMAKE_BINARY_DIR}/app/rdd2/casadi)

set(CASADI_FILES
  ${CASADI_DEST_DIR}/rdd2.c
  ${CASADI_DEST_DIR}/rdd2_loglinear.c
  ${CASADI_DEST_DIR}/bezier.c
  )

if (CONFIG_CEREBRI_RDD2_CASADI)
  list(APPEND SOURCE_FILES ${CASADI_FILES})
endif()

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/rdd2.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/rdd2.py ${CASADI_DEST_DIR}
  DEPENDS cyecca_install)

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/rdd2_loglinear.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/rdd2_loglinear.py ${CASADI_DEST_DIR}
  DEPENDS cyecca_install)

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/bezier.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/bezier.py ${CASADI_DEST_DIR}
  DEPENDS cyecca_install)

set_source_files_properties(
  ${SOURCE_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}"
  )

set_source_files_properties(
  ${CASADI_FILES}
  PROPERTIES COMPILE_FLAGS
  "${flags}\
  -Wno-unused-parameter\
  -Wno-missing-prototypes\
  -Wno-missing-declarations\
  -Wno-float-equal")

target_sources(app PRIVATE ${SOURCE_FILES})

target_include_directories(app SYSTEM BEFORE PRIVATE
  ${ZEPHYR_BASE}/include
  ${CMAKE_BINARY_DIR}
  )

# vi: ts=2 sw=2 et
