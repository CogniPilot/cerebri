# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SYNAPSE_ETHERNET ethernet)
add_subdirectory_ifdef(CONFIG_SYNAPSE_UART uart)
add_subdirectory_ifdef(CONFIG_SYNAPSE_VESC_CAN vesc_can)
add_subdirectory_ifdef(CONFIG_SYNAPSE_ZBUS zbus)
