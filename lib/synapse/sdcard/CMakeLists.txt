# Copyright (c) 2024, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_sdcard)

# we need to be able to include generated header files
zephyr_include_directories(
	)

zephyr_library_sources(
  src/main.c
  )

zephyr_link_libraries(ELMFAT)

#get_target_property(ELMFAT_INCLUDE_DIRS ELMFAT INCLUDE_DIRECTORIES)
#message(STATUS "includes elmfat: ${ELMFAT_INCLUDE_DIRS}")


#target_link_libraries(cerebri_synapse_sdcard ELMFAT)

#add_dependencies(cerebri_synapse_sdcard synapse_protobuf)


