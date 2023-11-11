# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_dream_sil)

# we need to be able to include generated header files
zephyr_include_directories()

set_source_files_properties(native_main.c
PROPERTIES COMPILE_DEFINITIONS
  "NO_POSIX_CHEATS;_BSD_SOURCE;_DEFAULT_SOURCE"
)

zephyr_library_sources(
  native_main.c
  zephyr_main.c
  )

add_dependencies(cerebri_dream_sil synapse_protobuf)

# vi: ts=2 sw=2 et
