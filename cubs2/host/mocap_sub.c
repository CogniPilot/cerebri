/*
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mocap_sub.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <zenoh-pico.h>

static z_owned_session_t g_session;
static z_owned_subscriber_t g_sub;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static cubs2_mocap_rigid_body_t g_latest;
static unsigned long g_frames;
static bool g_running;

static void data_handler(z_loaned_sample_t *sample, void *arg)
{
	(void)arg;
	const z_loaned_bytes_t *payload = z_sample_payload(sample);

	z_owned_slice_t slice;
	if (z_bytes_to_slice(payload, &slice) != Z_OK) {
		return;
	}

	const uint8_t *buf = z_slice_data(z_loan(slice));
	size_t len = z_slice_len(z_loan(slice));

	cubs2_mocap_rigid_body_t rb;
	if (cubs2_topic_fb_unpack_mocap_frame(buf, len, &rb)) {
		pthread_mutex_lock(&g_lock);
		g_latest = rb;
		g_frames++;
		pthread_mutex_unlock(&g_lock);
	}

	z_drop(z_move(slice));
}

int mocap_sub_start(const char *keyexpr, const char *mode, const char *connect)
{
	z_owned_config_t config;
	z_config_default(&config);

	zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode ? mode : "peer");
	if (connect != NULL && connect[0] != '\0') {
		zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, connect);
	}

	if (z_open(&g_session, z_move(config), NULL) != Z_OK) {
		fprintf(stderr, "mocap_sub: z_open failed\n");
		return -1;
	}

	/* Background tasks for reading the socket and keeping the session alive. */
	if (zp_start_read_task(z_loan_mut(g_session), NULL) != Z_OK ||
	    zp_start_lease_task(z_loan_mut(g_session), NULL) != Z_OK) {
		fprintf(stderr, "mocap_sub: failed to start zenoh tasks\n");
		z_drop(z_move(g_session));
		return -1;
	}

	z_view_keyexpr_t ke;
	if (z_view_keyexpr_from_str(&ke, keyexpr) != Z_OK) {
		fprintf(stderr, "mocap_sub: bad keyexpr '%s'\n", keyexpr);
		z_drop(z_move(g_session));
		return -1;
	}

	z_owned_closure_sample_t closure;
	z_closure_sample(&closure, data_handler, NULL, NULL);

	if (z_declare_subscriber(z_loan(g_session), &g_sub, z_loan(ke), z_move(closure), NULL) !=
	    Z_OK) {
		fprintf(stderr, "mocap_sub: z_declare_subscriber failed\n");
		z_drop(z_move(g_session));
		return -1;
	}

	g_running = true;
	fprintf(stderr, "mocap_sub: subscribed to '%s' (mode=%s%s%s)\n", keyexpr,
		mode ? mode : "peer", connect ? ", connect=" : "", connect ? connect : "");
	return 0;
}

bool mocap_sub_get_latest(cubs2_mocap_rigid_body_t *rb)
{
	pthread_mutex_lock(&g_lock);
	*rb = g_latest;
	pthread_mutex_unlock(&g_lock);
	return rb->valid;
}

unsigned long mocap_sub_frame_count(void)
{
	unsigned long n;
	pthread_mutex_lock(&g_lock);
	n = g_frames;
	pthread_mutex_unlock(&g_lock);
	return n;
}

void mocap_sub_stop(void)
{
	if (!g_running) {
		return;
	}
	z_drop(z_move(g_sub));
	z_drop(z_move(g_session));
	g_running = false;
}
