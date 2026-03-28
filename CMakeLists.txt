cmake_minimum_required(VERSION 3.20.0)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(WORKSPACE_ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(ZEPHYR_MODULES
  ${WORKSPACE_ROOT}/modules/hal/cmsis_6
  ${WORKSPACE_ROOT}/modules/hal/nxp
  ${WORKSPACE_ROOT}/modules/lib/zephyr_boards
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cerebri2 LANGUAGES C)

set(CEREBRI2_FLATBUFFER_SCHEMA_DIR
  ${WORKSPACE_ROOT}/modules/lib/synapse_msgs_fbs/fbs/cerebri2
)
set(CEREBRI2_FLATBUFFER_SCHEMA
  ${CEREBRI2_FLATBUFFER_SCHEMA_DIR}/cerebri2_topics.fbs
)
set(CEREBRI2_FLATBUFFER_LOG_SCHEMA
  ${CEREBRI2_FLATBUFFER_SCHEMA_DIR}/cerebri2_log.fbs
)
set(CEREBRI2_FLATBUFFER_GENERATED_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/generated/flatbuffers
)
set(CEREBRI2_FLATBUFFER_STAMP
  ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/.flatcc.stamp
)
set(CEREBRI2_FLATBUFFER_BFBS_STAMP
  ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/.bfbs.stamp
)
option(CEREBRI2_FLATBUFFER_CODEGEN
  "Generate FlatBuffer C headers and binary schemas into the build tree when flatcc is available"
  ON
)

find_program(CEREBRI2_FLATCC_EXECUTABLE NAMES flatcc)

if(NOT EXISTS ${CEREBRI2_FLATBUFFER_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer topic schema: ${CEREBRI2_FLATBUFFER_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(NOT EXISTS ${CEREBRI2_FLATBUFFER_LOG_SCHEMA})
  message(FATAL_ERROR
    "Missing FlatBuffer log schema: ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}. "
    "Run west update to fetch modules/lib/synapse_msgs_fbs."
  )
endif()

if(CEREBRI2_FLATBUFFER_CODEGEN AND CEREBRI2_FLATCC_EXECUTABLE)
  add_custom_command(
    OUTPUT ${CEREBRI2_FLATBUFFER_STAMP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CEREBRI2_FLATBUFFER_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI2_FLATBUFFER_SCHEMA}
            ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/cerebri2_topics.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
            ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/cerebri2_log.fbs
    COMMAND ${CEREBRI2_FLATCC_EXECUTABLE}
            -a
            -I${CEREBRI2_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI2_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI2_FLATBUFFER_SCHEMA}
    COMMAND ${CEREBRI2_FLATCC_EXECUTABLE}
            -a
            -I${CEREBRI2_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI2_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CEREBRI2_FLATBUFFER_STAMP}
    DEPENDS
      ${CEREBRI2_FLATBUFFER_SCHEMA}
      ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
    COMMENT "Generating FlatBuffer headers into ${CEREBRI2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cerebri2_flatbuffer_codegen DEPENDS ${CEREBRI2_FLATBUFFER_STAMP})
  add_custom_command(
    OUTPUT ${CEREBRI2_FLATBUFFER_BFBS_STAMP}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CEREBRI2_FLATBUFFER_GENERATED_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI2_FLATBUFFER_SCHEMA}
            ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/cerebri2_topics.fbs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
            ${CEREBRI2_FLATBUFFER_GENERATED_DIR}/cerebri2_log.fbs
    COMMAND ${CEREBRI2_FLATCC_EXECUTABLE}
            --schema
            -I${CEREBRI2_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI2_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI2_FLATBUFFER_SCHEMA}
    COMMAND ${CEREBRI2_FLATCC_EXECUTABLE}
            --schema
            -I${CEREBRI2_FLATBUFFER_SCHEMA_DIR}
            -o${CEREBRI2_FLATBUFFER_GENERATED_DIR}
            ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
    COMMAND ${CMAKE_COMMAND} -E touch ${CEREBRI2_FLATBUFFER_BFBS_STAMP}
    DEPENDS
      ${CEREBRI2_FLATBUFFER_SCHEMA}
      ${CEREBRI2_FLATBUFFER_LOG_SCHEMA}
    COMMENT "Generating FlatBuffer binary schemas into ${CEREBRI2_FLATBUFFER_GENERATED_DIR}"
    VERBATIM
  )

  add_custom_target(cerebri2_flatbuffer_schema_codegen
    DEPENDS ${CEREBRI2_FLATBUFFER_BFBS_STAMP}
  )
elseif(CEREBRI2_FLATBUFFER_CODEGEN)
  message(STATUS
    "flatcc not found; skipping FlatBuffer code generation and using local topic codec only"
  )
endif()

target_sources(app PRIVATE
  src/main.c
  src/topic_flatbuffer.c
  src/topic_shell.c
  drivers/nxp_flexio_dshot.c
)

if(TARGET cerebri2_flatbuffer_codegen)
  add_dependencies(app cerebri2_flatbuffer_codegen)
endif()

if(TARGET cerebri2_flatbuffer_schema_codegen)
  add_dependencies(app cerebri2_flatbuffer_schema_codegen)
endif()

target_include_directories(app PRIVATE
  include
  src
  ${CEREBRI2_FLATBUFFER_GENERATED_DIR}
  ${ZEPHYR_BASE}/../modules/hal/nxp/mcux/mcux-sdk-ng/drivers/flexio
)
