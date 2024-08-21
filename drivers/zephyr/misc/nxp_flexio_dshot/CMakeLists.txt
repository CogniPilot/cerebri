# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_NXP_FLEXIO_DSHOT
  nxp_flexio_dshot.c
)
