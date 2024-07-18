# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_sense_safety)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  main.c
  )

add_dependencies(cerebri_sense_safety synapse_pb)
