cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(ZEPHYR_MODULES
  ${WORKSPACE_ROOT}/modules/hal/cmsis
  ${WORKSPACE_ROOT}/modules/hal/cmsis_6
  ${WORKSPACE_ROOT}/modules/hal/nxp
  ${WORKSPACE_ROOT}/modules/lib/cmsis-dsp
  ${WORKSPACE_ROOT}/modules/lib/zephyr_boards
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cerebri LANGUAGES C)

set(CEREBRI_FLATBUFFER_SCHEMA_DIR
  ${WORKSPACE_ROOT}/modules/lib/synapse_msgs_fbs/fbs/cerebri
)
set(CEREBRI_FLATBUFFER_SCHEMA
  ${CEREBRI_FLATBUFFER_SCHEMA_DIR}/cerebri_topics.fbs
)
set(CEREBRI_FLATBUFFER_LOG_SCHEMA
  ${CEREBRI_FLATBUFFER_SCHEMA_DIR}/cerebri_log.fbs
)
set(CEREBRI_FLATBUFFER_SIL_SCHEMA
  ${CEREBRI_FLATBUFFER_SCHEMA_DIR}/cerebri_sil.fbs
)
set(CEREBRI_FLATBUFFER_GENERATED_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/generated/flatbuffers
)
set(CEREBRI_FLATBUFFER_STAMP
  ${CEREBRI_FLATBUFFER_GENERATED_DIR}/.flatcc.stamp
)
set(CEREBRI_FLATBUFFER_BFBS_STAMP
  ${CEREBRI_FLATBUFFER_GENERATED_DIR}/.bfbs.stamp
)
set(CEREBRI_FLATBUFFER_STAGE_STAMP
  ${CEREBRI_FLATBUFFER_GENERATED_DIR}/.schema-copy.stamp
)
option(CEREBRI_FLATBUFFER_CODEGEN
  "Generate FlatBuffer C headers and binary schemas into the build tree when flatcc is available"
  ON
)

find_program(CEREBRI_FLATCC_EXECUTABLE NAMES flatcc)

if(CEREBRI_FLATCC_EXECUTABLE)
  get_filename_component(CEREBRI_FLATCC_REALPATH ${CEREBRI_FLATCC_EXECUTABLE} REALPATH)
  get_filename_component(CEREBRI_FLATCC_BIN_DIR ${CEREBRI_FLATCC_REALPATH} DIRECTORY)
  get_filename_component(CEREBRI_FLATCC_PREFIX ${CEREBRI_FLATCC_BIN_DIR} DIRECTORY)
endif()

find_path(CEREBRI_FLATCC_INCLUDE_DIR
  NAMES flatcc/flatcc_builder.h
  HINTS ${CEREBRI_FLATCC_PREFIX}/include
)

if(NOT EXISTS ${CEREBRI_FLATBUFFER_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer topic schema: ${CEREBRI_FLATBUFFER_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CEREBRI_FLATBUFFER_LOG_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer log schema: ${CEREBRI_FLATBUFFER_LOG_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CEREBRI_FLATBUFFER_SIL_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer SITL schema: ${CEREBRI_FLATBUFFER_SIL_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(CEREBRI_FLATBUFFER_CODEGEN AND CEREBRI_FLATCC_EXECUTABLE)
  add_custom_command(
    OUTPUT ${CEREBRI_FLATBUFFER_STAGE_STAMP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CEREBRI_FLATBUFFER_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI_FLATBUFFER_SCHEMA}
            ${CEREBRI_FLATBUFFER_GENERATED_DIR}/cerebri_topics.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
            ${CEREBRI_FLATBUFFER_GENERATED_DIR}/cerebri_log.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
            ${CEREBRI_FLATBUFFER_GENERATED_DIR}/cerebri_sil.fbs
    COMMAND ${CMAKE_COMMAND} -E touch ${CEREBRI_FLATBUFFER_STAGE_STAMP}
    DEPENDS
      ${CEREBRI_FLATBUFFER_SCHEMA}
      ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
      ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Staging FlatBuffer schemas into ${CEREBRI_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_command(
    OUTPUT ${CEREBRI_FLATBUFFER_STAMP}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            -a
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_SCHEMA}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            -a
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            -a
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CEREBRI_FLATBUFFER_STAMP}
    DEPENDS
      ${CEREBRI_FLATBUFFER_STAGE_STAMP}
      ${CEREBRI_FLATBUFFER_SCHEMA}
      ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
      ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer headers into ${CEREBRI_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cerebri_flatbuffer_codegen DEPENDS ${CEREBRI_FLATBUFFER_STAMP})
  add_custom_command(
    OUTPUT ${CEREBRI_FLATBUFFER_BFBS_STAMP}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            --schema
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_SCHEMA}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            --schema
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CEREBRI_FLATCC_EXECUTABLE}
            --schema
            -I${CEREBRI_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CEREBRI_FLATBUFFER_BFBS_STAMP}
    DEPENDS
      ${CEREBRI_FLATBUFFER_STAGE_STAMP}
      ${CEREBRI_FLATBUFFER_SCHEMA}
      ${CEREBRI_FLATBUFFER_LOG_SCHEMA}
      ${CEREBRI_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer binary schemas into ${CEREBRI_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cerebri_flatbuffer_schema_codegen
    DEPENDS ${CEREBRI_FLATBUFFER_BFBS_STAMP}
  )
elseif(CEREBRI_FLATBUFFER_CODEGEN)
  message(FATAL_ERROR
    "flatcc is required to build cerebri because the firmware now compiles "
    "against generated FlatBuffer headers."
  )
endif()

if(NOT CEREBRI_FLATCC_INCLUDE_DIR)
  message(FATAL_ERROR
    "Missing flatcc headers. Expected flatcc/flatcc_builder.h near ${CEREBRI_FLATCC_EXECUTABLE}."
  )
endif()

if(TARGET cerebri_flatbuffer_codegen)
  add_dependencies(app cerebri_flatbuffer_codegen)
endif()

if(TARGET cerebri_flatbuffer_schema_codegen)
  add_dependencies(app cerebri_flatbuffer_schema_codegen)
endif()

target_include_directories(app PRIVATE
  include
  ${CEREBRI_FLATBUFFER_GENERATED_DIR}
  ${CEREBRI_FLATCC_INCLUDE_DIR}
  ${ZEPHYR_BASE}/../modules/hal/nxp/mcux/mcux-sdk-ng/drivers/flexio
)

add_subdirectory(src)
add_subdirectory(subsys)
add_subdirectory(drivers)
