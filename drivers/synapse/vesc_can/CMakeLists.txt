# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ACTUATOR_VESC_CAN actuator_vesc_can)
add_subdirectory_ifdef(CONFIG_VESC_CAN_status vesc_can_status)
