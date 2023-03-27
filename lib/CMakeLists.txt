# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CUSTOM_LIB custom_lib)
add_subdirectory_ifdef(CONFIG_SYNAPSE_ZBUS synapse_zbus)
