# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(synapse_zbus)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  zbus_topic_list.c
  uart_main.c
  ethernet_main.c
  channels.c
  )

add_dependencies(synapse_zbus synapse_protobuf)
