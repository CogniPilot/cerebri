# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_core_common)

zephyr_include_directories(src
  ${CMAKE_BINARY_DIR}
  )

zephyr_library_include_directories(src
  ${CMAKE_BINARY_DIR}
  )

set(CASADI_DEST_DIR ${CMAKE_BINARY_DIR}/lib/core/common/casadi)

set(CASADI_FILES
  ${CASADI_DEST_DIR}/common.c
  )

add_custom_command(OUTPUT ${CASADI_DEST_DIR}/common.c
  COMMAND ${CYECCA_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/common.py ${CASADI_DEST_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/casadi/common.py)

set_source_files_properties(
  ${CASADI_FILES}
  PROPERTIES COMPILE_FLAGS
  "-Wno-unused-parameter\
  -Wno-missing-prototypes\
  -Wno-missing-declarations\
  -Wno-float-equal")

zephyr_library_sources(
  src/common.c
  src/cerebri_log.c
  ${CASADI_FILES}
  )

add_dependencies(app cerebri_core_common)
