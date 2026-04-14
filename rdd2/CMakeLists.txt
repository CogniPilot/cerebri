cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(WORKSPACE_ROOT ${REPO_ROOT} DIRECTORY)
set(ZEPHYR_MODULES
  ${WORKSPACE_ROOT}/modules/hal/cmsis
  ${WORKSPACE_ROOT}/modules/hal/cmsis_6
  ${WORKSPACE_ROOT}/modules/hal/nxp
  ${WORKSPACE_ROOT}/modules/lib/cmsis-dsp
  ${WORKSPACE_ROOT}/modules/lib/zenoh-pico
  ${WORKSPACE_ROOT}/modules/lib/zros
  ${WORKSPACE_ROOT}/modules/lib/zephyr_boards
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rdd2 LANGUAGES C)

set(RDD2_FLATBUFFER_SCHEMA_DIR
  ${WORKSPACE_ROOT}/modules/lib/synapse_msgs_fbs/fbs/synapse
)
set(RDD2_FLATBUFFER_SCHEMA
  ${RDD2_FLATBUFFER_SCHEMA_DIR}/synapse_topics.fbs
)
set(RDD2_FLATBUFFER_LOG_SCHEMA
  ${RDD2_FLATBUFFER_SCHEMA_DIR}/synapse_log.fbs
)
set(RDD2_FLATBUFFER_SIL_SCHEMA
  ${RDD2_FLATBUFFER_SCHEMA_DIR}/synapse_sil.fbs
)
set(RDD2_FLATBUFFER_GENERATED_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/generated/flatbuffers
)
set(RDD2_FLATBUFFER_STAMP
  ${RDD2_FLATBUFFER_GENERATED_DIR}/.flatcc.stamp
)
set(RDD2_FLATBUFFER_BFBS_STAMP
  ${RDD2_FLATBUFFER_GENERATED_DIR}/.bfbs.stamp
)
set(RDD2_FLATBUFFER_STAGE_STAMP
  ${RDD2_FLATBUFFER_GENERATED_DIR}/.schema-copy.stamp
)
option(RDD2_FLATBUFFER_CODEGEN
  "Generate FlatBuffer C headers and binary schemas into the build tree when flatcc is available"
  ON
)

find_program(RDD2_FLATCC_EXECUTABLE NAMES flatcc)

if(RDD2_FLATCC_EXECUTABLE)
  get_filename_component(RDD2_FLATCC_REALPATH ${RDD2_FLATCC_EXECUTABLE} REALPATH)
  get_filename_component(RDD2_FLATCC_BIN_DIR ${RDD2_FLATCC_REALPATH} DIRECTORY)
  get_filename_component(RDD2_FLATCC_PREFIX ${RDD2_FLATCC_BIN_DIR} DIRECTORY)
endif()

find_path(RDD2_FLATCC_INCLUDE_DIR
  NAMES flatcc/flatcc_builder.h
  HINTS ${RDD2_FLATCC_PREFIX}/include
)

if(NOT EXISTS ${RDD2_FLATBUFFER_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer topic schema: ${RDD2_FLATBUFFER_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${RDD2_FLATBUFFER_LOG_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer log schema: ${RDD2_FLATBUFFER_LOG_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${RDD2_FLATBUFFER_SIL_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer SITL schema: ${RDD2_FLATBUFFER_SIL_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(RDD2_FLATBUFFER_CODEGEN AND RDD2_FLATCC_EXECUTABLE)
  add_custom_command(
    OUTPUT ${RDD2_FLATBUFFER_STAGE_STAMP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RDD2_FLATBUFFER_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${RDD2_FLATBUFFER_SCHEMA}
            ${RDD2_FLATBUFFER_GENERATED_DIR}/synapse_topics.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${RDD2_FLATBUFFER_LOG_SCHEMA}
            ${RDD2_FLATBUFFER_GENERATED_DIR}/synapse_log.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${RDD2_FLATBUFFER_SIL_SCHEMA}
            ${RDD2_FLATBUFFER_GENERATED_DIR}/synapse_sil.fbs
    COMMAND ${CMAKE_COMMAND} -E touch ${RDD2_FLATBUFFER_STAGE_STAMP}
    DEPENDS
      ${RDD2_FLATBUFFER_SCHEMA}
      ${RDD2_FLATBUFFER_LOG_SCHEMA}
      ${RDD2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Staging FlatBuffer schemas into ${RDD2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_command(
    OUTPUT ${RDD2_FLATBUFFER_STAMP}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            -a
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_SCHEMA}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            -a
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            -a
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${RDD2_FLATBUFFER_STAMP}
    DEPENDS
      ${RDD2_FLATBUFFER_STAGE_STAMP}
      ${RDD2_FLATBUFFER_SCHEMA}
      ${RDD2_FLATBUFFER_LOG_SCHEMA}
      ${RDD2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer headers into ${RDD2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(rdd2_flatbuffer_codegen DEPENDS ${RDD2_FLATBUFFER_STAMP})
  add_custom_command(
    OUTPUT ${RDD2_FLATBUFFER_BFBS_STAMP}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            --schema
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_SCHEMA}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            --schema
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${RDD2_FLATCC_EXECUTABLE}
            --schema
            -I${RDD2_FLATBUFFER_SCHEMA_DIR}
            -o${RDD2_FLATBUFFER_GENERATED_DIR}
            ${RDD2_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${RDD2_FLATBUFFER_BFBS_STAMP}
    DEPENDS
      ${RDD2_FLATBUFFER_STAGE_STAMP}
      ${RDD2_FLATBUFFER_SCHEMA}
      ${RDD2_FLATBUFFER_LOG_SCHEMA}
      ${RDD2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer binary schemas into ${RDD2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(rdd2_flatbuffer_schema_codegen
    DEPENDS ${RDD2_FLATBUFFER_BFBS_STAMP}
  )
elseif(RDD2_FLATBUFFER_CODEGEN)
  message(FATAL_ERROR
    "flatcc is required to build RDD2 because the firmware now compiles "
    "against generated FlatBuffer headers."
  )
endif()

if(NOT RDD2_FLATCC_INCLUDE_DIR)
  message(FATAL_ERROR
    "Missing flatcc headers. Expected flatcc/flatcc_builder.h near ${RDD2_FLATCC_EXECUTABLE}."
  )
endif()

if(TARGET rdd2_flatbuffer_codegen)
  add_dependencies(app rdd2_flatbuffer_codegen)
endif()

if(TARGET rdd2_flatbuffer_schema_codegen)
  add_dependencies(app rdd2_flatbuffer_schema_codegen)
endif()

target_include_directories(app PRIVATE
  include
  ${RDD2_FLATBUFFER_GENERATED_DIR}
  ${RDD2_FLATCC_INCLUDE_DIR}
  ${ZEPHYR_BASE}/../modules/hal/nxp/mcux/mcux-sdk-ng/drivers/flexio
)

add_subdirectory(src)
add_subdirectory(subsys)
add_subdirectory(drivers)
