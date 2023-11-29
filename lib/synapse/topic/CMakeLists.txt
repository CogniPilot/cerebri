# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_topic)

zephyr_library_include_directories(include)
zephyr_include_directories(include)

zephyr_library_sources(
  src/synapse_shell_print.c
  src/synapse_topic.c
  src/synapse_topic_list.c
  )


add_dependencies(cerebri_synapse_topic synapse_protobuf)
