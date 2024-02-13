#!/usr/bin/env python3

top = """# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

menuconfig CEREBRI_ACTUATE_PWM
  bool "PWM"
  depends on PWM
  depends on ZROS
  help
    This option enables pwm actuators

if CEREBRI_ACTUATE_PWM

config CEREBRI_ACTUATE_PWM_NUMBER
  int "Number of PWM actuators"
  default 1
  range 1 8
  help
    Defines number of PWM actuators 1-8
"""

middle = """
###############################################################################
menu "PWM_{i:d}"
  visible if CEREBRI_ACTUATE_PWM_NUMBER > {i:d}

config CEREBRI_ACTUATE_PWM_OUTPUT_{i:d}
  string "Aliased output device PWM {i:d}"
  default "aux{i:d}"
  help
    Aliased name of PWM output device used for PWM {i:d}

config CEREBRI_ACTUATE_PWM_USE_NANO_SECONDS_{i:d}
  bool "Use nano seconds for PWM {i:d}"
  help
    Enable to use nanoseconds for the pulse widths of PWM {i:d}.

config CEREBRI_ACTUATE_PWM_PULSE_MIN_{i:d}
  int "Min pulse width PWM {i:d}"
  default 1100
  range 0 9999999
  help
    Minimum allowed pulse width in micro
    (or nano if selected) seconds for PWM {i:d}

config CEREBRI_ACTUATE_PWM_PULSE_MAX_{i:d}
  int "Max pulse width PWM {i:d}"
  default 1900
  range 1 10000000
  help
    Maximum allowed pulse width in micro
    (or nano if selected) seconds for PWM {i:d}

config CEREBRI_ACTUATE_PWM_PULSE_CENTER_{i:d}
  int "Center of pulse width PWM {i:d}"
  default 1500
  range 2 9999998
  help
    Center of pulse width in micro
    (or nano if selected) seconds for PWM {i:d}
    This is the value it also uses on startup before
    getting message signals or arming.

config CEREBRI_ACTUATE_PWM_INDEX_{i:d}
  int "actuator index for PWM {i:d}"
  default {i:d}
  help
    The actuator message index to use for position,
    velocity or nomalized fields

menuconfig CEREBRI_ACTUATE_PWM_LINEAR_{i:d}
  bool "LINEAR PWM {i:d}"
  help
    Allows you to take in an actuator position or velocity
    index and calculate [rad, m] or [rad/s, m/s] -> PWM
    micro (or nano if selected) seconds with
    a linear approximation using slope
    intercept formula to best fit your actuator.

if CEREBRI_ACTUATE_PWM_LINEAR_{i:d}

config CEREBRI_ACTUATE_PWM_USE_POS_{i:d}
  bool "Use actuator position PWM {i:d}"
  help
    Use actuator message position instead of velocity for
    CEREBRI_ACTUATE_PWM_INDEX_{i:d}

config CEREBRI_ACTUATE_PWM_LINEAR_M_DIV_{i:d}
  int "divisor to divide linear slope M"
  default 1000
  help
    The divisor value to divide slope constant M value of the
    linearized PWM/[vel or pos] by [1, 10, 100, 1000 (default),
    10000, 100000, ...]

config CEREBRI_ACTUATE_PWM_LINEAR_M_{i:d}
  int "linear slope M (divisor scaled)"
  default 1000
  help
    The slope constant M value of the linearized PWM/[vel or pos]
    divided by the divisor CEREBRI_ACTUATE_PWM_LINEAR_M_DIV (default 1000)

config CEREBRI_ACTUATE_PWM_LINEAR_B_DIV_{i:d}
  int "divisor to divide linear intercept B"
  default 1000
  help
    The divisor value to divide intercept constant B of the
    linearized PWM/[vel or pos] by [1, 10, 100, 1000 (default), 
    10000, 100000, ...]

config CEREBRI_ACTUATE_PWM_LINEAR_B_{i:d}
  int "linear intercept B (divisor scaled)"
  default 1500000
  help
    The intercept constant B value of the linearized PWM/[vel or pos]
    divided by the divisor CEREBRI_ACTUATE_PWM_LINEAR_B_DIV (default 1000)

endif #CEREBRI_ACTUATE_PWM_LINEAR_{i:d}

endmenu #PWM_{i:d}
"""

end = """
module = CEREBRI_ACTUATE_PWM
module-str = actuate_pwm
source "subsys/logging/Kconfig.template.log_config"

endif #CEREBRI_ACTUATE_PWM
"""

with open('Kconfig.gen', 'w') as f:
    f.write(top)
    for i in range(8):
        f.write(middle.format(**locals()))
    f.write(end)

