cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
get_filename_component(WORKSPACE_ROOT ${REPO_ROOT} DIRECTORY)
set(ZEPHYR_MODULES
  ${WORKSPACE_ROOT}/modules/hal/cmsis
  ${WORKSPACE_ROOT}/modules/hal/cmsis_6
  ${WORKSPACE_ROOT}/modules/hal/nxp
  ${WORKSPACE_ROOT}/modules/lib/cmsis-dsp
  ${WORKSPACE_ROOT}/modules/lib/zephyr_boards
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
set(CUBS2_TARGET_NAME "cubs2_sil" CACHE STRING "CUBS2 build target/profile name")
project(${CUBS2_TARGET_NAME} LANGUAGES C)

set(CUBS2_FLATBUFFER_SCHEMA_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set(CUBS2_FLATBUFFER_SCHEMA
  ${CUBS2_FLATBUFFER_SCHEMA_DIR}/synapse_topics.fbs
)
set(CUBS2_FLATBUFFER_MOCAP_SCHEMA
  ${CUBS2_FLATBUFFER_SCHEMA_DIR}/synapse_mocap.fbs
)
set(CUBS2_FLATBUFFER_LOG_SCHEMA
  ${CUBS2_FLATBUFFER_SCHEMA_DIR}/synapse_log.fbs
)
set(CUBS2_FLATBUFFER_SIL_SCHEMA
  ${CUBS2_FLATBUFFER_SCHEMA_DIR}/synapse_sil.fbs
)
set(CUBS2_FLATBUFFER_GENERATED_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/generated/flatbuffers
)
set(CUBS2_FLATBUFFER_STAMP
  ${CUBS2_FLATBUFFER_GENERATED_DIR}/.flatcc.stamp
)
set(CUBS2_FLATBUFFER_BFBS_STAMP
  ${CUBS2_FLATBUFFER_GENERATED_DIR}/.bfbs.stamp
)
set(CUBS2_FLATBUFFER_STAGE_STAMP
  ${CUBS2_FLATBUFFER_GENERATED_DIR}/.schema-copy.stamp
)
option(CUBS2_FLATBUFFER_CODEGEN
  "Generate FlatBuffer C headers and binary schemas into the build tree when flatcc is available"
  ON
)

find_program(CUBS2_FLATCC_EXECUTABLE NAMES flatcc)

if(CUBS2_FLATCC_EXECUTABLE)
  get_filename_component(CUBS2_FLATCC_REALPATH ${CUBS2_FLATCC_EXECUTABLE} REALPATH)
  get_filename_component(CUBS2_FLATCC_BIN_DIR ${CUBS2_FLATCC_REALPATH} DIRECTORY)
  get_filename_component(CUBS2_FLATCC_PREFIX ${CUBS2_FLATCC_BIN_DIR} DIRECTORY)
endif()

find_path(CUBS2_FLATCC_INCLUDE_DIR
  NAMES flatcc/flatcc_builder.h
  HINTS ${CUBS2_FLATCC_PREFIX}/include
)

if(NOT EXISTS ${CUBS2_FLATBUFFER_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer topic schema: ${CUBS2_FLATBUFFER_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CUBS2_FLATBUFFER_MOCAP_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer mocap schema: ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CUBS2_FLATBUFFER_LOG_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer log schema: ${CUBS2_FLATBUFFER_LOG_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CUBS2_FLATBUFFER_SIL_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer SITL schema: ${CUBS2_FLATBUFFER_SIL_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(CUBS2_FLATBUFFER_CODEGEN AND CUBS2_FLATCC_EXECUTABLE)
  add_custom_command(
    OUTPUT ${CUBS2_FLATBUFFER_STAGE_STAMP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CUBS2_FLATBUFFER_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CUBS2_FLATBUFFER_SCHEMA}
            ${CUBS2_FLATBUFFER_GENERATED_DIR}/synapse_topics.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
            ${CUBS2_FLATBUFFER_GENERATED_DIR}/synapse_mocap.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CUBS2_FLATBUFFER_LOG_SCHEMA}
            ${CUBS2_FLATBUFFER_GENERATED_DIR}/synapse_log.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CUBS2_FLATBUFFER_SIL_SCHEMA}
            ${CUBS2_FLATBUFFER_GENERATED_DIR}/synapse_sil.fbs
    COMMAND ${CMAKE_COMMAND} -E touch ${CUBS2_FLATBUFFER_STAGE_STAMP}
    DEPENDS
      ${CUBS2_FLATBUFFER_SCHEMA}
      ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
      ${CUBS2_FLATBUFFER_LOG_SCHEMA}
      ${CUBS2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Staging FlatBuffer schemas into ${CUBS2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_command(
    OUTPUT ${CUBS2_FLATBUFFER_STAMP}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            -a
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            -a
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            -a
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            -a
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CUBS2_FLATBUFFER_STAMP}
    DEPENDS
      ${CUBS2_FLATBUFFER_STAGE_STAMP}
      ${CUBS2_FLATBUFFER_SCHEMA}
      ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
      ${CUBS2_FLATBUFFER_LOG_SCHEMA}
      ${CUBS2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer headers into ${CUBS2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cubs2_flatbuffer_codegen DEPENDS ${CUBS2_FLATBUFFER_STAMP})
  add_custom_command(
    OUTPUT ${CUBS2_FLATBUFFER_BFBS_STAMP}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            --schema
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            --schema
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            --schema
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CUBS2_FLATCC_EXECUTABLE}
            --schema
            -I${CUBS2_FLATBUFFER_SCHEMA_DIR}
            -o${CUBS2_FLATBUFFER_GENERATED_DIR}
            ${CUBS2_FLATBUFFER_SIL_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CUBS2_FLATBUFFER_BFBS_STAMP}
    DEPENDS
      ${CUBS2_FLATBUFFER_STAGE_STAMP}
      ${CUBS2_FLATBUFFER_SCHEMA}
      ${CUBS2_FLATBUFFER_MOCAP_SCHEMA}
      ${CUBS2_FLATBUFFER_LOG_SCHEMA}
      ${CUBS2_FLATBUFFER_SIL_SCHEMA}
    COMMENT "Generating FlatBuffer binary schemas into ${CUBS2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cubs2_flatbuffer_schema_codegen
    DEPENDS ${CUBS2_FLATBUFFER_BFBS_STAMP}
  )
elseif(CUBS2_FLATBUFFER_CODEGEN)
  message(FATAL_ERROR
    "flatcc is required to build CUBS2 because the firmware now compiles "
    "against generated FlatBuffer headers."
  )
endif()

if(NOT CUBS2_FLATCC_INCLUDE_DIR)
  message(FATAL_ERROR
    "Missing flatcc headers. Expected flatcc/flatcc_builder.h near ${CUBS2_FLATCC_EXECUTABLE}."
  )
endif()

if(TARGET cubs2_flatbuffer_codegen)
  add_dependencies(app cubs2_flatbuffer_codegen)
endif()

if(TARGET cubs2_flatbuffer_schema_codegen)
  add_dependencies(app cubs2_flatbuffer_schema_codegen)
endif()

target_include_directories(app PRIVATE
  include
  ${CUBS2_FLATBUFFER_GENERATED_DIR}
  ${CUBS2_FLATCC_INCLUDE_DIR}
  ${ZEPHYR_BASE}/../modules/hal/nxp/mcux/mcux-sdk-ng/drivers/flexio
)

add_subdirectory(src)
add_subdirectory(subsys)
add_subdirectory(drivers)
