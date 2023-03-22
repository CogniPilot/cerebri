# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(synaptic_gap)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  main.c
  channels.c
  )

add_dependencies(synaptic_gap synapse_protobuf)
