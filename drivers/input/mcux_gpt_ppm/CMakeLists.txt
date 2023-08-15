# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(input_mcux_gpt_ppm)


# we need to be able to include generated header files
zephyr_include_directories()

zephyr_library_sources(
  input_mcux_gpt_ppm.c
  )

