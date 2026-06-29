/*
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mocap_sub.h"

#include <stdio.h>
#include <string.h>

int mocap_sub_start(const char *keyexpr, const char *mode, const char *connect)
{
	(void)keyexpr;
	(void)mode;
	(void)connect;

	fprintf(stderr,
		"mocap_sub: disabled; use zephyr native_sim with csyn for live mocap\n");
	return -1;
}

bool mocap_sub_get_latest(cubs2_mocap_rigid_body_t *rb)
{
	if (rb != NULL) {
		memset(rb, 0, sizeof(*rb));
	}
	return false;
}

unsigned long mocap_sub_frame_count(void)
{
	return 0UL;
}

void mocap_sub_stop(void)
{
}
