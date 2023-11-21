# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(cerebri_synapse_zros)

zephyr_library_include_directories(include)
zephyr_include_directories(include)

zephyr_library_sources(
	src/zros_node.c
	src/zros_node_manager.c
	src/zros_pub.c
	src/zros_pull_sub.c
	src/zros_push_sub.c
	src/zros_topic.c
	src/zros_node.c
  )
