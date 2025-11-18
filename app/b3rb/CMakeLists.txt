#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(b3rb LANGUAGES C)

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

if (CONFIG_CEREBRI_B3RB_FSM)
  list(APPEND SOURCE_FILES src/fsm.c)
endif()

if (CONFIG_CEREBRI_B3RB_ESTIMATE)
  list(APPEND SOURCE_FILES src/estimate.c)
endif()

if (CONFIG_CEREBRI_B3RB_MIXING)
  list(APPEND SOURCE_FILES src/mixing.c)
endif()

if (CONFIG_CEREBRI_B3RB_LIGHTING)
  list(APPEND SOURCE_FILES src/lighting.c)
endif()

if (CONFIG_CEREBRI_B3RB_COMMAND)
  list(APPEND SOURCE_FILES src/command.c)
endif()

if (CONFIG_CEREBRI_B3RB_POSITION)
  list(APPEND SOURCE_FILES
    src/position.c)
endif()

if (CONFIG_CEREBRI_B3RB_VELOCITY)
  list(APPEND SOURCE_FILES
    src/velocity.c)
endif()

set(CASADI_DEST_DIR ${CMAKE_BINARY_DIR}/app/b3rb/casadi)

set(CASADI_FILES
  ${CASADI_DEST_DIR}/b3rb.c
  )

if (CONFIG_CEREBRI_B3RB_CASADI)
  list(APPEND SOURCE_FILES
    ${CASADI_DEST_DIR}/b3rb.c
    )
endif()

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/b3rb.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/b3rb.py ${CASADI_DEST_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/b3rb.py)

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

if(CONFIG_BOARD STREQUAL "mr_navq95a")
  # Relocate code to ITCM
  zephyr_linker_sources(ITCM_SECTION boards/mr_navq95a_itcm_linker.ld)
  zephyr_code_relocate(LIBRARY nanopb LOCATION ITCM_TEXT)
  # Relocate log ring buffer to SDRAM
  zephyr_linker_sources(SECTIONS boards/navq95.ld)
endif()

# vi: ts=2 sw=2 et
