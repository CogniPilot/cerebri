# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(synapse_zbus)

# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  syn_pub_sub.c
  channels.c
  )

add_dependencies(synapse_zbus synapse_protobuf)
