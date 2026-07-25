# Copyright (c) 2024, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_eth_tx)

# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  src/main.c
  src/proto/udp_tx.c
  )

add_dependencies(cerebri_synapse_eth_tx synapse_protobuf)
