/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cognipilot_sitl_rc

#include <zephyr/device.h>

static int sitl_rc_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define RDD2_SITL_RC_INIT(inst)                                                                    \
	DEVICE_DT_INST_DEFINE(inst, sitl_rc_init, NULL, NULL, NULL, POST_KERNEL,                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL)

DT_INST_FOREACH_STATUS_OKAY(RDD2_SITL_RC_INIT)
