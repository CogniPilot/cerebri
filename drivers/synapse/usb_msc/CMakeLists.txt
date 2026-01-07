# Copyright (c) 2024, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_usb_msc)

# Export public API header
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Include log_sdcard header for auto stop/resume functionality
if(CONFIG_CEREBRI_SYNAPSE_LOG_SDCARD)
  zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../log_sdcard/include)
endif()

zephyr_library_sources(
  src/main.c
)
