# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SYNAPSE_ZBUS zbus)
add_subdirectory_ifdef(CONFIG_SYNAPSE_ETHERNET ethernet)
add_subdirectory_ifdef(CONFIG_SYNAPSE_TOPIC topic)
add_subdirectory_ifdef(CONFIG_SYNAPSE_VESC_CAN vesc_can)
