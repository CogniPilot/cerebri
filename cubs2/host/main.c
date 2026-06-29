/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * cubs2_host: native fixed-wing outer-loop controller.
 *
 *   cubs2_host  --(57600 serial: 0xFF-framed RC)-->  ppm_bridge Arduino  --> PPM --> Tx
 *
 * This legacy POSIX port keeps the controller/serial smoke test buildable.
 * Live mocap and Zenoh-facing transport moved to zephyr native_sim with csyn.
 */
#include "CubControl_FixedWingOuterLoop.h"
#include "topic_flatbuffer.h"
#include "ppm_serial.h"
#include "mocap_sub.h"

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Generated-model p[] slot aliases (rumoca embedded-c). Gains, waypoints and
 * the rest of the parameters are baked into the model by
 * CubControl_FixedWingOuterLoop_init(); only the pose inputs and the control
 * outputs are exchanged each cycle. */
#define P_X          CUBCONTROL_FIXEDWINGOUTERLOOP_P_x
#define P_Y          CUBCONTROL_FIXEDWINGOUTERLOOP_P_y
#define P_Z          CUBCONTROL_FIXEDWINGOUTERLOOP_P_z
#define P_ROLL       CUBCONTROL_FIXEDWINGOUTERLOOP_P_roll
#define P_PITCH      CUBCONTROL_FIXEDWINGOUTERLOOP_P_pitch
#define P_YAW        CUBCONTROL_FIXEDWINGOUTERLOOP_P_yaw
#define P_AILERON    CUBCONTROL_FIXEDWINGOUTERLOOP_P_aileron
#define P_ELEVATOR   CUBCONTROL_FIXEDWINGOUTERLOOP_P_elevator
#define P_THROTTLE   CUBCONTROL_FIXEDWINGOUTERLOOP_P_throttle
#define P_RUDDER     CUBCONTROL_FIXEDWINGOUTERLOOP_P_rudder
#define P_STABILIZER CUBCONTROL_FIXEDWINGOUTERLOOP_P_stabilizer
#define P_CURRENT_WP CUBCONTROL_FIXEDWINGOUTERLOOP_P_current_wp
#define P_AIRBORNE   CUBCONTROL_FIXEDWINGOUTERLOOP_P_airborne

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static float clampf_local(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static uint16_t pwm_from_centered_stick(float value, bool inverted)
{
	const float sign = inverted ? -1.0f : 1.0f;
	return (uint16_t)(1500.0f + 500.0f * sign * clampf_local(value, -1.0f, 1.0f));
}

static uint16_t pwm_from_throttle(float value)
{
	return (uint16_t)(1000.0f + 1000.0f * clampf_local(value, 0.0f, 1.0f));
}

static void quat_to_euler(const cubs2_mocap_rigid_body_t *mocap, float *roll, float *pitch,
			  float *yaw)
{
	const float qw = mocap->qw, qx = mocap->qx, qy = mocap->qy, qz = mocap->qz;
	const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
	const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
	const float sinp = 2.0f * (qw * qy - qz * qx);
	const float siny_cosp = 2.0f * (qw * qz + qx * qy);
	const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

	*roll = atan2f(sinr_cosp, cosr_cosp);
	*pitch = asinf(clampf_local(sinp, -1.0f, 1.0f));
	*yaw = atan2f(siny_cosp, cosy_cosp);
}

static void map_input(CubControl_FixedWingOuterLoop_t *m, const cubs2_mocap_rigid_body_t *mocap)
{
	float roll, pitch, yaw;
	quat_to_euler(mocap, &roll, &pitch, &yaw);
	m->p[P_X] = mocap->x;
	m->p[P_Y] = mocap->y;
	m->p[P_Z] = mocap->z;
	m->p[P_ROLL] = roll;
	m->p[P_PITCH] = pitch;
	m->p[P_YAW] = yaw;
}

/* PPM channel order: aileron, elevator(inverted), throttle, rudder, stabilizer/mode. */
static void map_output(const CubControl_FixedWingOuterLoop_t *m, uint16_t ch[PPM_NUM_CHANNELS])
{
	ch[0] = pwm_from_centered_stick((float)m->p[P_AILERON], false);
	ch[1] = pwm_from_centered_stick((float)m->p[P_ELEVATOR], true);
	ch[2] = pwm_from_throttle((float)m->p[P_THROTTLE]);
	ch[3] = pwm_from_centered_stick((float)m->p[P_RUDDER], false);
	ch[4] = (uint16_t)clampf_local((float)m->p[P_STABILIZER], 1000.0f, 2000.0f);
}

static const char *env_or(const char *name, const char *fallback)
{
	const char *v = getenv(name);
	return (v && v[0]) ? v : fallback;
}

int main(int argc, char **argv)
{
	const char *ppm_dev = (argc > 1) ? argv[1] : env_or("CUBS2_PPM_DEVICE", "/dev/ttyACM0");

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	CubControl_FixedWingOuterLoop_t model;
	CubControl_FixedWingOuterLoop_init(&model);

	int fd = ppm_serial_open(ppm_dev);
	if (fd < 0) {
		return 1;
	}
	fprintf(stderr, "cubs2_host: PPM serial open on %s @ 57600\n", ppm_dev);

	if (mocap_sub_start(NULL, NULL, NULL) != 0) {
		fprintf(stderr, "cubs2_host: continuing without mocap (failsafe output)\n");
	}

	const real_t dt = 0.01; /* 100 Hz, matches embedded default */
	const long period_ns = 10L * 1000L * 1000L;

	uint16_t ch[PPM_NUM_CHANNELS];
	unsigned long ticks = 0;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	while (!g_stop) {
		cubs2_mocap_rigid_body_t mocap;
		bool have_mocap = mocap_sub_get_latest(&mocap);

		if (have_mocap) {
			map_input(&model, &mocap);
			/* one 100 Hz discrete control step (snapshots pre(), runs the tick) */
			CubControl_FixedWingOuterLoop_step(&model, dt);
			map_output(&model, ch);
		} else {
			/* Failsafe until first mocap frame: throttle low, sticks centered. */
			ch[0] = 1500;
			ch[1] = 1500;
			ch[2] = 1000;
			ch[3] = 1500;
			ch[4] = 1900;
		}

		if (ppm_serial_send(fd, ch) != 0) {
			fprintf(stderr, "cubs2_host: serial write failed\n");
		}

		if ((ticks % 100UL) == 0UL) {
			fprintf(stderr,
				"t=%.1fs frames=%lu mocap=%d | A=%u E=%u T=%u R=%u M=%u | wp=%d air=%d\n",
				(double)ticks * 0.01, mocap_sub_frame_count(), (int)have_mocap,
				ch[0], ch[1], ch[2], ch[3], ch[4],
				(int)model.p[P_CURRENT_WP], (int)model.p[P_AIRBORNE]);
		}
		ticks++;

		/* Drift-free 100 Hz: sleep until the next absolute deadline. */
		next.tv_nsec += period_ns;
		while (next.tv_nsec >= 1000000000L) {
			next.tv_nsec -= 1000000000L;
			next.tv_sec += 1;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

	fprintf(stderr, "\ncubs2_host: stopping, sending failsafe\n");
	ch[0] = 1500; ch[1] = 1500; ch[2] = 1000; ch[3] = 1500; ch[4] = 1900;
	ppm_serial_send(fd, ch);
	mocap_sub_stop();
	ppm_serial_close(fd);
	return 0;
}
