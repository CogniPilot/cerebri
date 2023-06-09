# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
	DESTINATION bin
	RENAME cerebri_${CONFIG_CEREBRI_APP}_${BOARD} OPTIONAL)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.exe
	DESTINATION bin
	RENAME cerebri_${CONFIG_CEREBRI_APP}_${BOARD} OPTIONAL)
