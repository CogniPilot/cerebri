# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

# Note that here, we're adding CMAKE_SOURCE_DIR to the include path for nanopb.
# This is needed because the below call to nanopb_generate_cpp() is using
# 'RELPATH .'
set(NANOPB_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}")
nanopb_generate_cpp(proto_sources proto_headers RELPATH .
    vector3.proto
    twist.proto
)
zephyr_library_named(synapse_pb)

# we need to be able to include generated header files
zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

zephyr_library_sources(
  ${proto_sources}
  )
