# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(estimate_attitude)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  main.c
  casadi/casadi_mrp.c
  )

add_dependencies(estimate_attitude synapse_protobuf)
