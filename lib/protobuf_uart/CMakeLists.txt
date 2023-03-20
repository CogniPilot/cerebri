# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(protobuf_uart)

zephyr_library_sources(
  main.c
)

add_dependencies(protobuf_uart synapse_pb)
