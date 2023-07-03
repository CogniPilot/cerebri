# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SYNAPSE_VESC_CAN_STATUS vesc_can_status)
add_subdirectory_ifdef(CONFIG_SENSE_INA226 ina226)
