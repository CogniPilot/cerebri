# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(sense_ina226)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  main.c
  )

add_dependencies(sense_ina226 synapse_protobuf)
