# Copyright (c) 2025 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_NXP_FLEXIO_DSHOT nxp_flexio_dshot)
add_subdirectory_ifdef(CONFIG_NXP_FLEXIO_UART nxp_flexio_uart)
