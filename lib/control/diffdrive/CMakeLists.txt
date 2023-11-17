# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_control_diffdrive)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  mixing.c
  manual.c
  fsm.c
  velocity.c
  position.c
  casadi/rover.c
  )

add_dependencies(cerebri_control_diffdrive synapse_protobuf)
