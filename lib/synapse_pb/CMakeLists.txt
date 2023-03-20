# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

# Note that here, we're adding CMAKE_SOURCE_DIR to the include path for nanopb.
# This is needed because the below call to nanopb_generate_cpp() is using
# 'RELPATH .'
set(NANOPB_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB PROTO_FILES "*.proto")
nanopb_generate_cpp(PROTO_SOURCES PROTO_HEADERS RELPATH .
  ${PROTO_FILES}
)
zephyr_library_named(synapse_pb)

# we need to be able to include generated header files
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

zephyr_library_sources(
  ${PROTO_SOURCES}
  )

add_dependencies(app synapse_pb)
