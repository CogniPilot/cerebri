/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Test mocap publisher: emits synthetic MocapFrame flatbuffers on
 * "synapse/mocap/frame", byte-identical to synapse_qualisys_bridge, so the
 * cubs2_host control loop can be exercised without real hardware.
 *
 * The rigid body climbs (z: 0 -> 5 m) then circles, so cubs2_host should
 * transition takeoff -> airborne and drive non-failsafe RC outputs.
 *
 * Env: CUBS2_ZENOH_MODE (peer|client, default peer), CUBS2_ZENOH_CONNECT,
 *      CUBS2_MOCAP_KEY (default synapse/mocap/frame), CUBS2_PUB_HZ (default 50).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <zenoh-pico.h>
#include "synapse_mocap_builder.h"

static const char *env_or(const char *n, const char *d)
{
	const char *v = getenv(n);
	return (v && v[0]) ? v : d;
}

int main(void)
{
	const char *mode = env_or("CUBS2_ZENOH_MODE", "peer");
	const char *connect = getenv("CUBS2_ZENOH_CONNECT");
	const char *key = env_or("CUBS2_MOCAP_KEY", "synapse/mocap/frame");
	double hz = atof(env_or("CUBS2_PUB_HZ", "50"));
	if (hz <= 0) {
		hz = 50;
	}

	z_owned_config_t cfg;
	z_config_default(&cfg);
	const char *listen = getenv("CUBS2_ZENOH_LISTEN");
	zp_config_insert(z_loan_mut(cfg), Z_CONFIG_MODE_KEY, mode);
	if (connect && connect[0]) {
		zp_config_insert(z_loan_mut(cfg), Z_CONFIG_CONNECT_KEY, connect);
	}
	if (listen && listen[0]) {
		zp_config_insert(z_loan_mut(cfg), Z_CONFIG_LISTEN_KEY, listen);
	}

	z_owned_session_t s;
	if (z_open(&s, z_move(cfg), NULL) != Z_OK) {
		fprintf(stderr, "mocap_pub: z_open failed (no zenoh transport?)\n");
		return 1;
	}
	zp_start_read_task(z_loan_mut(s), NULL);
	zp_start_lease_task(z_loan_mut(s), NULL);

	z_view_keyexpr_t ke;
	z_view_keyexpr_from_str(&ke, key);

	fprintf(stderr, "mocap_pub: publishing on '%s' (mode=%s) at %.0f Hz\n", key, mode, hz);

	const struct timespec period = {.tv_sec = 0, .tv_nsec = (long)(1e9 / hz)};
	for (unsigned long k = 0;; k++) {
		double t = (double)k / hz;
		/* Hold on the ground for 3 s (z=0, at wp1) so a subscriber that
		 * joins late still observes the takeoff -> airborne crossing, then
		 * climb and circle. */
		const double t_ground = 3.0;
		double tc = t > t_ground ? t - t_ground : 0.0;
		float z = (float)fmin(5.0, tc * 2.0);           /* climb to 5 m */
		float x = (float)(-4.0 + (tc > 0 ? 10.0 * (1.0 - cos(0.2 * tc)) : 0.0));
		float y = (float)(-5.0 + (tc > 0 ? 7.0 * sin(0.2 * tc) : 0.0));
		float yaw = (float)(0.2 * tc);

		flatcc_builder_t b;
		flatcc_builder_init(&b);
		synapse_topic_MocapFrame_start_as_root(&b);
		synapse_topic_MocapFrame_timestamp_us_add(&b, (uint64_t)(t * 1e6));
		synapse_topic_MocapFrame_frame_number_add(&b, (uint32_t)k);
		synapse_topic_MocapRigidBodySample_vec_start(&b);
		synapse_topic_Vec3f_t pos = {x, y, z};
		synapse_topic_Quaternionf_t att = {0.0f, 0.0f, sinf(yaw / 2.0f), cosf(yaw / 2.0f)};
		synapse_topic_MocapRigidBodySample_t rb;
		memset(&rb, 0, sizeof(rb));
		rb.id = 1;
		rb.position = pos;
		rb.attitude = att;
		rb.residual = 0.0f;
		rb.tracking_valid = 1;
		synapse_topic_MocapRigidBodySample_vec_push(&b, &rb);
		synapse_topic_MocapFrame_rigid_bodies_add(
			&b, synapse_topic_MocapRigidBodySample_vec_end(&b));
		synapse_topic_MocapFrame_end_as_root(&b);

		size_t n;
		uint8_t *buf = flatcc_builder_finalize_buffer(&b, &n);
		z_owned_bytes_t payload;
		z_bytes_copy_from_buf(&payload, buf, n);
		z_put(z_loan(s), z_loan(ke), z_move(payload), NULL);
		flatcc_builder_clear(&b);
		free(buf);

		nanosleep(&period, NULL);
	}

	z_drop(z_move(s));
	return 0;
}
