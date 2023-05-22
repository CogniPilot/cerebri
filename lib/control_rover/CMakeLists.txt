# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(control_rover)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  main.c
  casadi/rover.c
  )

add_dependencies(control_rover synapse_protobuf)
