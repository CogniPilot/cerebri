# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_COMMUNICATE_CAN can)
add_subdirectory_ifdef(CONFIG_COMMUNICATE_VESC_CAN vesc_can)
add_subdirectory_ifdef(CONFIG_COMMUNICATE_SYNAPSE synapse)
