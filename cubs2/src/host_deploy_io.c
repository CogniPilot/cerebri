/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "host_deploy_io.h"
#include "control_io.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_CUBS2_HOST_DEPLOY_JOYSTICK)
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#endif

LOG_MODULE_REGISTER(cubs2_host_deploy_io, LOG_LEVEL_INF);

#define HOST_DEPLOY_JOYSTICK_STACK_SIZE 2048
#define HOST_DEPLOY_JOYSTICK_PRIORITY 5

static struct k_mutex g_input_mutex;
static cubs2_mocap_rigid_body_t g_latest_mocap;
static synapse_topic_RcChannels16_t g_latest_rc;
static bool g_have_mocap;
static bool g_have_rc;

#if defined(CONFIG_CUBS2_HOST_DEPLOY_JOYSTICK)
K_THREAD_STACK_DEFINE(g_joystick_stack, HOST_DEPLOY_JOYSTICK_STACK_SIZE);
static struct k_thread g_joystick_thread;
#endif

void cubs2_host_deploy_io_put_rc(const synapse_topic_RcChannels16_t *rc, bool valid)
{
	if (rc == NULL) {
		return;
	}

	k_mutex_lock(&g_input_mutex, K_FOREVER);
	g_latest_rc = *rc;
	g_have_rc = valid;
	k_mutex_unlock(&g_input_mutex);

	if (valid) {
		cubs2_control_input_trigger();
	}
}

void cubs2_host_deploy_io_put_mocap_payload(const uint8_t *buf, size_t len)
{
	cubs2_mocap_rigid_body_t mocap;
	const float mocap_mm_to_m = 0.001f;

	if (buf == NULL || !cubs2_topic_fb_unpack_mocap_frame(buf, len, &mocap)) {
		return;
	}

	mocap.x *= mocap_mm_to_m;
	mocap.y *= mocap_mm_to_m;
	mocap.z *= mocap_mm_to_m;

	k_mutex_lock(&g_input_mutex, K_FOREVER);
	g_latest_mocap = mocap;
	g_have_mocap = mocap.valid;
	k_mutex_unlock(&g_input_mutex);

	cubs2_control_input_trigger();
}

bool cubs2_host_deploy_io_get_mocap(cubs2_mocap_rigid_body_t *mocap)
{
	bool have_mocap;

	if (mocap == NULL) {
		return false;
	}

	k_mutex_lock(&g_input_mutex, K_FOREVER);
	*mocap = g_latest_mocap;
	have_mocap = g_have_mocap;
	k_mutex_unlock(&g_input_mutex);

	return have_mocap;
}

bool cubs2_host_deploy_io_get_rc(synapse_topic_RcChannels16_t *rc)
{
	bool have_rc;

	if (rc == NULL) {
		return false;
	}

	k_mutex_lock(&g_input_mutex, K_FOREVER);
	*rc = g_latest_rc;
	have_rc = g_have_rc;
	k_mutex_unlock(&g_input_mutex);

	return have_rc;
}

#if defined(CONFIG_CUBS2_HOST_DEPLOY_JOYSTICK)
static void host_deploy_store_rc(const synapse_topic_RcChannels16_t *rc)
{
	k_mutex_lock(&g_input_mutex, K_FOREVER);
	g_latest_rc = *rc;
	g_have_rc = true;
	k_mutex_unlock(&g_input_mutex);
}

static int32_t joystick_axis_to_pwm(int16_t value, bool inverted)
{
	int32_t centered = 1500;
	int32_t scaled = ((int32_t)value * 500) / 32767;

	if (inverted) {
		scaled = -scaled;
	}

	return centered + scaled;
}

static int32_t joystick_throttle_to_pwm(int16_t value)
{
	/* Linux joystick axes are usually -32767..32767. Map high stick to 2000. */
	return 1000 + (((int32_t)(32767 - value) * 1000) / 65534);
}

static void joystick_apply_axis(synapse_topic_RcChannels16_t *rc, uint8_t axis, int16_t value)
{
	switch (axis) {
	case CONFIG_CUBS2_JOYSTICK_AILERON_AXIS:
		rc->ch0 = joystick_axis_to_pwm(value, false);
		break;
	case CONFIG_CUBS2_JOYSTICK_ELEVATOR_AXIS:
		rc->ch1 = joystick_axis_to_pwm(value, true);
		break;
	case CONFIG_CUBS2_JOYSTICK_THROTTLE_AXIS:
		rc->ch2 = joystick_throttle_to_pwm(value);
		break;
	case CONFIG_CUBS2_JOYSTICK_RUDDER_AXIS:
		rc->ch3 = joystick_axis_to_pwm(value, false);
		break;
	default:
		break;
	}
}

static void joystick_apply_button(synapse_topic_RcChannels16_t *rc, uint8_t button, int16_t value)
{
	if (button == CONFIG_CUBS2_JOYSTICK_AUTO_BUTTON) {
		rc->ch5 = value ? 2000 : 1000;
	}
}

static void joystick_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		const char *device = CONFIG_CUBS2_JOYSTICK_DEVICE;
		int fd = open(device, O_RDONLY);

		if (fd < 0) {
			LOG_WRN("joystick open failed for %s: %d", device, errno);
			k_sleep(K_SECONDS(1));
			continue;
		}

		LOG_INF("joystick opened on %s", device);

		synapse_topic_RcChannels16_t rc = {
			.ch0 = 1500,
			.ch1 = 1500,
			.ch2 = 1000,
			.ch3 = 1500,
			.ch4 = 1900,
			.ch5 = 1000,
		};

		host_deploy_store_rc(&rc);

		while (true) {
			struct js_event event;
			ssize_t n = read(fd, &event, sizeof(event));

			if (n != sizeof(event)) {
				LOG_WRN("joystick read failed for %s: %d", device, errno);
				break;
			}

			event.type &= ~JS_EVENT_INIT;
			if (event.type == JS_EVENT_AXIS) {
				joystick_apply_axis(&rc, event.number, event.value);
				host_deploy_store_rc(&rc);
			} else if (event.type == JS_EVENT_BUTTON) {
				joystick_apply_button(&rc, event.number, event.value);
				host_deploy_store_rc(&rc);
			}
		}

		close(fd);
		k_sleep(K_MSEC(250));
	}
}
#endif

int cubs2_host_deploy_io_init(void)
{
	int rc = 0;

	k_mutex_init(&g_input_mutex);

#if defined(CONFIG_CUBS2_HOST_DEPLOY_JOYSTICK)
	k_thread_create(&g_joystick_thread, g_joystick_stack,
			K_THREAD_STACK_SIZEOF(g_joystick_stack), joystick_thread,
			NULL, NULL, NULL, HOST_DEPLOY_JOYSTICK_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&g_joystick_thread, "cubs2_joystick");
#endif

	return rc;
}
