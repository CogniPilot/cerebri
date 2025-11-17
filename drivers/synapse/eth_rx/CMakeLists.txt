# Copyright (c) 2024, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_eth_rx)

# we need to be able to include generated header files
zephyr_include_directories()

set(SOURCE_FILES
  src/main.c
  )

if (NOT CONFIG_CEREBRI_SYNAPSE_RPMSG)
  list(APPEND SOURCE_FILES src/proto/udp_rx.c)
endif()

zephyr_library_sources(${SOURCE_FILES})

add_dependencies(cerebri_synapse_eth_rx synapse_pb)
