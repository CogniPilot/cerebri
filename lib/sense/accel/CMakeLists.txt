# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_sense_accel)


zephyr_include_directories()

zephyr_library_sources(
  main.c
  )

add_dependencies(cerebri_sense_accel synapse_pb)
add_dependencies(cerebri_sense_accel cerebri_core_common)
