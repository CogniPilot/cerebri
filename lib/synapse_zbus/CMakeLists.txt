# Copyright (c) 2023, CongiPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(synapse_zbus)

add_dependencies(synapse_zbus synapse_pb)

zephyr_library_sources(
  channels.c
  )
