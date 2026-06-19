/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zenoh-pico subscriber for the MocapFrame flatbuffer published by
 * CogniPilot/synapse_qualisys_bridge on key "synapse/mocap/frame".
 */
#ifndef CUBS2_HOST_MOCAP_SUB_H_
#define CUBS2_HOST_MOCAP_SUB_H_

#include <stdbool.h>
#include "topic_flatbuffer.h" /* cubs2_mocap_rigid_body_t */

/*
 * Start a background zenoh-pico session and subscribe to the mocap frame key.
 *
 * keyexpr: zenoh key to subscribe to (e.g. "synapse/mocap/frame").
 * mode:    "peer" or "client" (NULL -> "peer").
 * connect: optional endpoint to connect to (e.g. "tcp/127.0.0.1:7447"),
 *          or NULL to rely on multicast scouting.
 *
 * Returns 0 on success.
 */
int mocap_sub_start(const char *keyexpr, const char *mode, const char *connect);

/* Copy the most-recent rigid body into *rb. Returns rb->valid. */
bool mocap_sub_get_latest(cubs2_mocap_rigid_body_t *rb);

/* Total frames received since start (for diagnostics). */
unsigned long mocap_sub_frame_count(void);

void mocap_sub_stop(void);

#endif /* CUBS2_HOST_MOCAP_SUB_H_ */
