# Copyright (c) 2024, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_log_sdcard)

# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  src/main.c
  src/writer.c
  )

if(CONFIG_FAT_FILESYSTEM_ELM)
  zephyr_link_libraries(ELMFAT)
endif()

add_dependencies(cerebri_synapse_log_sdcard synapse_pb cerebri_synapse_rpmsg)
