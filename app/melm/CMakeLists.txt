#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(melm LANGUAGES C)

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

if (CONFIG_CEREBRI_MELM_FSM)
  list(APPEND SOURCE_FILES src/fsm.c)
endif()

if (CONFIG_CEREBRI_MELM_ESTIMATE)
  list(APPEND SOURCE_FILES src/estimate.c)
endif()

if (CONFIG_CEREBRI_MELM_MIXING)
  list(APPEND SOURCE_FILES src/mixing.c)
endif()

if (CONFIG_CEREBRI_MELM_LIGHTING)
  list(APPEND SOURCE_FILES src/lighting.c)
endif()

if (CONFIG_CEREBRI_MELM_COMMAND)
  list(APPEND SOURCE_FILES src/command.c)
endif()

if (CONFIG_CEREBRI_MELM_POSITION)
  list(APPEND SOURCE_FILES
    src/position.c)
endif()

if (CONFIG_CEREBRI_MELM_VELOCITY)
  list(APPEND SOURCE_FILES
    src/velocity.c)
endif()

set(CASADI_DEST_DIR ${CMAKE_BINARY_DIR}/app/melm/casadi)

set(CASADI_FILES
  ${CASADI_DEST_DIR}/melm.c
  )

if (CONFIG_CEREBRI_MELM_CASADI)
  list(APPEND SOURCE_FILES
    ${CASADI_DEST_DIR}/melm.c
    )
endif()

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/melm.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/melm.py ${CASADI_DEST_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/melm.py)

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

target_include_directories(app SYSTEM BEFORE PRIVATE ${ZEPHYR_BASE}/include ${CMAKE_BINARY_DIR})

# vi: ts=2 sw=2 et
