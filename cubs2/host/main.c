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
	m->x = mocap->x;
	m->y = mocap->y;
	m->z = mocap->z;
	m->roll = roll;
	m->pitch = pitch;
	m->yaw = yaw;
}

/* PPM channel order: aileron(inverted), elevator(inverted), throttle, rudder, stabilizer/mode. */
static void map_output(const CubControl_FixedWingOuterLoop_t *m, uint16_t ch[PPM_NUM_CHANNELS])
{
	ch[0] = pwm_from_centered_stick((float)m->aileron, true);
	ch[1] = pwm_from_centered_stick((float)m->elevator, true);
	ch[2] = pwm_from_throttle((float)m->throttle);
	ch[3] = pwm_from_centered_stick((float)m->rudder, false);
	ch[4] = (uint16_t)clampf_local((float)m->stabilizer, 1000.0f, 2000.0f);
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
	model.dt = CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_S;

	int fd = ppm_serial_open(ppm_dev);
	if (fd < 0) {
		return 1;
	}
	fprintf(stderr, "cubs2_host: PPM serial open on %s @ 57600\n", ppm_dev);

	if (mocap_sub_start(NULL, NULL, NULL) != 0) {
		fprintf(stderr, "cubs2_host: continuing without mocap (failsafe output)\n");
	}

	const long period_ns = CUBCONTROL_FIXEDWINGOUTERLOOP_PERIOD_NS;

	uint16_t ch[PPM_NUM_CHANNELS];
	unsigned long ticks = 0;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	while (!g_stop) {
		cubs2_mocap_rigid_body_t mocap;
		bool have_mocap = mocap_sub_get_latest(&mocap);

		if (have_mocap) {
			map_input(&model, &mocap);
			CubControl_FixedWingOuterLoop_step(&model);
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
				(double)ticks * (double)model.dt, mocap_sub_frame_count(), (int)have_mocap,
				ch[0], ch[1], ch[2], ch[3], ch[4],
				(int)model.current_wp, (int)model.airborne);
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
