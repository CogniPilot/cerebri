# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CEREBRI_CORE_WORKQUEUES workqueues)
add_subdirectory_ifdef(CONFIG_CEREBRI_CORE_COMMON common)
