# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_zbus)

# we need to be able to include generated header files
include_directories(
	${CMAKE_CURRENT_BINARY_DIR}/modules
	)
zephyr_include_directories()

zephyr_library_sources(
  syn_pub_sub.c
  channels.c
  )

add_dependencies(cerebri_synapse_zbus synapse_protobuf)
