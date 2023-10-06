#-------------------------------------------------------------------------------
# Zephyr Cerebri Application
#
# Copyright (c) 2023 CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mrbuggy3 LANGUAGES C)

target_compile_options(app PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

set(SOURCE_FILES
  src/main.c
  )

target_sources(app PRIVATE ${SOURCE_FILES})

#add_custom_target(install_files ALL
#	DEPENDS ${ZEPHYR_FINAL_EXECUTABLE}
#)

#add_custom_command(
#    TARGET install_files
#    POST_BUILD
#    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target install
#    )
