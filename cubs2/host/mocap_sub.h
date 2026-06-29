/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Legacy host mocap hook.
 *
 * Live mocap transport moved to the Zephyr native_sim + csyn path. This host
 * shim is retained only so cubs2_host can still build without Zenoh.
 */
#ifndef CUBS2_HOST_MOCAP_SUB_H_
#define CUBS2_HOST_MOCAP_SUB_H_

#include <stdbool.h>
#include "topic_flatbuffer.h" /* cubs2_mocap_rigid_body_t */

/* Returns -1; live mocap now belongs to the csyn/native_sim path. */
int mocap_sub_start(const char *keyexpr, const char *mode, const char *connect);

/* Copy the most-recent rigid body into *rb. Returns rb->valid. */
bool mocap_sub_get_latest(cubs2_mocap_rigid_body_t *rb);

/* Total frames received since start (for diagnostics). */
unsigned long mocap_sub_frame_count(void);

void mocap_sub_stop(void);

#endif /* CUBS2_HOST_MOCAP_SUB_H_ */
