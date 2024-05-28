# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_core_common)

zephyr_include_directories()
zephyr_library_include_directories(src)

zephyr_library_sources(
  src/common.c
  src/casadi/gen/common.c
  )
