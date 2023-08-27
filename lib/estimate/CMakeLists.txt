# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CEREBRI_ESTIMATE_ATTITUDE attitude)
add_subdirectory_ifdef(CONFIG_CEREBRI_ESTIMATE_ROVER2D rover2d)
