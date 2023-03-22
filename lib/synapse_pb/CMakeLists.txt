# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

FetchContent_Populate(
  synapse_pb
  GIT_REPOSITORY https://github.com/CogniPilot/synapse_pb.git
  GIT_TAG 0e2a88c65dfacb16478cd2963489f8ed288183ab
  SOURCE_DIR synapse_pb
  QUIET
)

file(GLOB PROTO_FILES "${CMAKE_CURRENT_BINARY_DIR}/synapse_pb/*.proto")
message(STATUS "files:${PROTO_FILES}")

set(NANOPB_IMPORT_DIRS "${CMAKE_CURRENT_BINARY_DIR}/synapse_pb")
nanopb_generate_cpp(PROTO_SOURCES PROTO_HEADERS RELPATH ./synapse_pb ${PROTO_FILES})
message(STATUS "srcs:${PROTO_SOURCES}")

zephyr_library_named(synapse_pb)


# we need to be able to include generated header files
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})

zephyr_library_sources(
  main.c
  channels.c
  ${PROTO_SOURCES}
  ${PROTO_HEADERS}
  )

add_dependencies(app synapse_pb)
