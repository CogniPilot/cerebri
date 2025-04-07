#
# Copyright (c) 2025 CogniPilot Foundation
# Copyright 2025 NXP
#
# SPDX-License-Identifier: Apache-2.0
#

# Add custom linker section to relocate memory to DDR and OCRAM
zephyr_linker_sources_ifdef(CONFIG_SOC_MIMX9596_M7
  SECTIONS navq95.ld)