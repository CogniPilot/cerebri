#
# Copyright (c) 2025 CogniPilot Foundation
# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

# Add custom linker section to relocate framebuffers to PSRAM
zephyr_linker_sources_ifdef(CONFIG_SOC_MIMX9596_M7
  SECTIONS ddr.ld)