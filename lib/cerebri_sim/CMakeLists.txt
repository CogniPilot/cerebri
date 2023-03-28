# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_sim)

# we need to be able to include generated header files
zephyr_include_directories()

set(SOURCE_FILES
  main.c
  )

set_source_files_properties(${SOURCE_FILES}
PROPERTIES COMPILE_DEFINITIONS
  "NO_POSIX_CHEATS;_BSD_SOURCE;_DEFAULT_SOURCE"
)

zephyr_library_sources(
  ${SOURCE_FILES}
  )

add_dependencies(cerebri_sim synapse_protobuf)

# vi: ts=2 sw=2 et
