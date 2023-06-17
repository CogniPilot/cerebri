# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ACTUATE_PWM pwm)
add_subdirectory_ifdef(CONFIG_ACTUATE_VESC_CAN	vesc_can)
