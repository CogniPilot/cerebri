# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

zephyr_include_directories(include)

find_program(PIPX pipx)
if (NOT PIPX)
	message(FATAL_ERROR "install pipx via: sudo apt install pipx")
endif()

find_program(POETRY poetry)
if (NOT POETRY)
	message(FATAL_ERROR "install poetry via: pipx install poetry")
endif()
set(CEREBRI_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CYECCA_PYTHON  ${CEREBRI_ROOT_DIR}/scripts/cyecca_python)

execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cyecca_install)

add_subdirectory(lib)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/../install)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	RENAME cerebri_${CONFIG_CEREBRI_APP_NAME}_${BOARD} OPTIONAL)

install(PROGRAMS ${CMAKE_BINARY_DIR}/zephyr/zephyr.exe
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	RENAME cerebri_${CONFIG_CEREBRI_APP_NAME}_${BOARD} OPTIONAL)

install(FILES ${CMAKE_SOURCE_DIR}/../../scripts/setup.sh
	DESTINATION ${CMAKE_INSTALL_PREFIX})

set(CPACK_PACKAGE_NAME cerebri_${CONFIG_CEREBRI_APP_NAME})
set(CPACK_GENERATOR "DEB;TGZ")
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
set(CPACK_SYSTEM_NAME "${BOARD}")
set(CPACK_PACKAGE_VERSION_MAJOR "${CONFIG_CEREBRI_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${CONFIG_CEREBRI_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${CONFIG_CEREBRI_VERSION_PATCH}")
set(CPACK_PACKAGE_CONTACT cogni@cognipilot.com)

include(CPack)
