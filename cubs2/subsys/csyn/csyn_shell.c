/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "csyn.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#define CSYN_HEX_BYTES_PER_LINE 16U

enum csyn_watch_mode {
	CSYN_WATCH_NONE = 0,
	CSYN_WATCH_ECHO,
	CSYN_WATCH_HZ,
};

struct csyn_watch {
	const struct shell *sh;
	enum cubs2_csyn_topic_id topic;
	enum csyn_watch_mode mode;
	uint32_t period_ms;
	uint32_t last_generation;
	int64_t last_ms;
	bool active;
};

static struct csyn_watch g_csyn_watch;
static uint8_t g_csyn_shell_buf[CUBS2_CSYN_TOPIC_MAX_SIZE];

K_SEM_DEFINE(g_csyn_watch_sem, 0, 1);

static void csyn_watch_thread(void *p0, void *p1, void *p2);
static void csyn_topic_dynamic_get(size_t idx, struct shell_static_entry *entry);

K_THREAD_DEFINE(g_csyn_watch_tid, CONFIG_CUBS2_CSYN_SHELL_THREAD_STACK_SIZE,
		csyn_watch_thread, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
SHELL_DYNAMIC_CMD_CREATE(sub_csyn_topic_names, csyn_topic_dynamic_get);

static void csyn_topic_dynamic_get(size_t idx, struct shell_static_entry *entry)
{
	const struct cubs2_csyn_topic_info *info;

	if (idx >= cubs2_csyn_topic_count()) {
		entry->syntax = NULL;
		return;
	}

	info = cubs2_csyn_topic_info((enum cubs2_csyn_topic_id)idx);
	entry->syntax = info->name;
	entry->handler = NULL;
	entry->subcmd = NULL;
	entry->help = NULL;
}

static int parse_u32_arg(const char *arg, uint32_t *out)
{
	char *end = NULL;
	long value = strtol(arg, &end, 10);

	if (*arg == '\0' || *end != '\0' || value <= 0 || value > INT32_MAX) {
		return -EINVAL;
	}

	*out = (uint32_t)value;
	return 0;
}

static const char *kind_name(enum cubs2_csyn_topic_kind kind)
{
	switch (kind) {
	case CUBS2_CSYN_KIND_FLATBUFFER:
		return "flatbuffer";
	case CUBS2_CSYN_KIND_STRUCT:
		return "struct";
	case CUBS2_CSYN_KIND_OPAQUE:
	default:
		return "opaque";
	}
}

static void print_hex(const struct shell *sh, const uint8_t *buf, size_t len)
{
	for (size_t offset = 0U; offset < len; offset += CSYN_HEX_BYTES_PER_LINE) {
		char line[(CSYN_HEX_BYTES_PER_LINE * 3U) + 1U];
		size_t line_len = 0U;
		size_t chunk_len = MIN(CSYN_HEX_BYTES_PER_LINE, len - offset);

		for (size_t i = 0U; i < chunk_len; i++) {
			line_len += (size_t)snprintk(&line[line_len], sizeof(line) - line_len,
						     "%02x%s", buf[offset + i],
						     (i + 1U < chunk_len) ? " " : "");
		}

		shell_print(sh, "+0x%04x: %s", (unsigned int)offset, line);
	}
}

static void print_rc(const struct shell *sh, const uint8_t *buf, size_t len)
{
	const synapse_topic_RcChannels16_t *rc = (const synapse_topic_RcChannels16_t *)buf;

	if (len != sizeof(*rc)) {
		shell_error(sh, "rc: invalid sample size %u", (unsigned int)len);
		return;
	}

	shell_print(sh, "rc ch0-7=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]",
		    (long)rc->ch0, (long)rc->ch1, (long)rc->ch2, (long)rc->ch3,
		    (long)rc->ch4, (long)rc->ch5, (long)rc->ch6, (long)rc->ch7);
	shell_print(sh, "rc ch8-15=[%4ld %4ld %4ld %4ld %4ld %4ld %4ld %4ld]",
		    (long)rc->ch8, (long)rc->ch9, (long)rc->ch10, (long)rc->ch11,
		    (long)rc->ch12, (long)rc->ch13, (long)rc->ch14, (long)rc->ch15);
}

static int csyn_topic_echo_once(const struct shell *sh, enum cubs2_csyn_topic_id topic)
{
	const struct cubs2_csyn_topic_info *info = cubs2_csyn_topic_info(topic);
	size_t len;
	uint32_t generation;

	if (info == NULL) {
		return -EINVAL;
	}

	if (!cubs2_csyn_topic_copy(topic, g_csyn_shell_buf, sizeof(g_csyn_shell_buf), &len,
				   &generation)) {
		shell_print(sh, "%s: no samples", info->name);
		return 0;
	}

	shell_print(sh, "%s gen=%u len=%u type=%s key=%s", info->name,
		    (unsigned int)generation, (unsigned int)len, info->type_name, info->keyexpr);

	if (topic == CUBS2_CSYN_TOPIC_CONTROL_OUTPUT) {
		print_rc(sh, g_csyn_shell_buf, len);
	} else {
		print_hex(sh, g_csyn_shell_buf, len);
	}

	return 0;
}

static void csyn_watch_stop(void)
{
	unsigned int key = irq_lock();

	g_csyn_watch.active = false;
	g_csyn_watch.mode = CSYN_WATCH_NONE;
	g_csyn_watch.topic = CUBS2_CSYN_TOPIC_INVALID;
	g_csyn_watch.sh = NULL;
	g_csyn_watch.period_ms = 0U;
	g_csyn_watch.last_generation = 0U;
	g_csyn_watch.last_ms = 0;

	irq_unlock(key);

	k_sem_give(&g_csyn_watch_sem);
}

static void csyn_watch_start(const struct shell *sh, enum cubs2_csyn_topic_id topic,
			     enum csyn_watch_mode mode, uint32_t period_ms)
{
	unsigned int key = irq_lock();

	g_csyn_watch.sh = sh;
	g_csyn_watch.topic = topic;
	g_csyn_watch.mode = mode;
	g_csyn_watch.period_ms = period_ms;
	g_csyn_watch.last_generation = cubs2_csyn_topic_generation(topic);
	g_csyn_watch.last_ms = k_uptime_get();
	g_csyn_watch.active = true;

	irq_unlock(key);

	k_sem_give(&g_csyn_watch_sem);
}

static void csyn_watch_thread(void *p0, void *p1, void *p2)
{
	struct csyn_watch watch;

	ARG_UNUSED(p0);
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	while (true) {
		(void)k_sem_take(&g_csyn_watch_sem, K_FOREVER);

		while (true) {
			unsigned int key = irq_lock();

			watch = g_csyn_watch;
			irq_unlock(key);

			if (!watch.active || watch.sh == NULL ||
			    watch.topic == CUBS2_CSYN_TOPIC_INVALID) {
				break;
			}

			if (watch.mode == CSYN_WATCH_ECHO) {
				(void)csyn_topic_echo_once(watch.sh, watch.topic);
				if (k_sem_take(&g_csyn_watch_sem, K_MSEC(watch.period_ms)) == 0) {
					continue;
				}
			} else if (watch.mode == CSYN_WATCH_HZ) {
				uint32_t generation_now;
				int64_t now_ms;
				const struct cubs2_csyn_topic_info *info;

				if (k_sem_take(&g_csyn_watch_sem, K_MSEC(watch.period_ms)) == 0) {
					continue;
				}

				key = irq_lock();
				watch = g_csyn_watch;
				irq_unlock(key);

				if (!watch.active || watch.sh == NULL ||
				    watch.topic == CUBS2_CSYN_TOPIC_INVALID) {
					break;
				}

				info = cubs2_csyn_topic_info(watch.topic);
				generation_now = cubs2_csyn_topic_generation(watch.topic);
				now_ms = k_uptime_get();
				if (now_ms <= watch.last_ms) {
					now_ms = watch.last_ms + 1;
				}

				shell_print(watch.sh, "%s: %u samples in %lld ms = %0.2f Hz",
					    info->name,
					    (unsigned int)(generation_now -
							   watch.last_generation),
					    (long long)(now_ms - watch.last_ms),
					    ((double)(generation_now -
						      watch.last_generation) * 1000.0) /
						    (double)(now_ms - watch.last_ms));

				key = irq_lock();
				if (g_csyn_watch.active && g_csyn_watch.mode == CSYN_WATCH_HZ &&
				    g_csyn_watch.topic == watch.topic) {
					g_csyn_watch.last_generation = generation_now;
					g_csyn_watch.last_ms = now_ms;
				}
				irq_unlock(key);
			} else {
				break;
			}
		}
	}
}

static int cmd_csyn_topic_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	for (size_t i = 0U; i < cubs2_csyn_topic_count(); i++) {
		const struct cubs2_csyn_topic_info *info =
			cubs2_csyn_topic_info((enum cubs2_csyn_topic_id)i);
		shell_print(sh, "%-16s %-12s max=%u key=%s", info->name,
			    kind_name(info->kind), (unsigned int)info->max_size, info->keyexpr);
	}

	return 0;
}

static int cmd_csyn_topic_info(const struct shell *sh, size_t argc, char **argv)
{
	enum cubs2_csyn_topic_id topic = cubs2_csyn_topic_parse(argv[1]);
	const struct cubs2_csyn_topic_info *info = cubs2_csyn_topic_info(topic);

	ARG_UNUSED(argc);

	if (info == NULL) {
		shell_error(sh, "unknown topic: %s", argv[1]);
		return -ENOENT;
	}

	shell_print(sh, "name=%s", info->name);
	shell_print(sh, "keyexpr=%s", info->keyexpr);
	shell_print(sh, "type=%s", info->type_name);
	shell_print(sh, "kind=%s", kind_name(info->kind));
	shell_print(sh, "max_size=%u", (unsigned int)info->max_size);
	shell_print(sh, "generation=%u", (unsigned int)cubs2_csyn_topic_generation(topic));

	return 0;
}

static int cmd_csyn_topic_echo(const struct shell *sh, size_t argc, char **argv)
{
	enum cubs2_csyn_topic_id topic = cubs2_csyn_topic_parse(argv[1]);
	uint32_t period_ms = 0U;
	int rc;

	if (topic == CUBS2_CSYN_TOPIC_INVALID) {
		shell_error(sh, "unknown topic: %s", argv[1]);
		return -ENOENT;
	}

	if (argc >= 3) {
		rc = parse_u32_arg(argv[2], &period_ms);
		if (rc != 0) {
			shell_error(sh, "period_ms must be a positive integer");
			return rc;
		}
	}

	if (period_ms == 0U) {
		return csyn_topic_echo_once(sh, topic);
	}

	csyn_watch_start(sh, topic, CSYN_WATCH_ECHO, period_ms);
	shell_print(sh, "echoing %s every %u ms; use 'csyn topic stop' to stop",
		    cubs2_csyn_topic_info(topic)->name, (unsigned int)period_ms);

	return 0;
}

static int cmd_csyn_topic_hz(const struct shell *sh, size_t argc, char **argv)
{
	enum cubs2_csyn_topic_id topic = cubs2_csyn_topic_parse(argv[1]);
	uint32_t period_ms = 1000U;
	int rc;

	if (topic == CUBS2_CSYN_TOPIC_INVALID) {
		shell_error(sh, "unknown topic: %s", argv[1]);
		return -ENOENT;
	}

	if (argc >= 3) {
		rc = parse_u32_arg(argv[2], &period_ms);
		if (rc != 0) {
			shell_error(sh, "window_ms must be a positive integer");
			return rc;
		}
	}

	csyn_watch_start(sh, topic, CSYN_WATCH_HZ, period_ms);
	shell_print(sh, "measuring %s every %u ms; use 'csyn topic stop' to stop",
		    cubs2_csyn_topic_info(topic)->name, (unsigned int)period_ms);

	return 0;
}

static int cmd_csyn_topic_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	csyn_watch_stop();
	shell_print(sh, "csyn topic watcher stopped");

	return 0;
}

static int cmd_csyn_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "csyn: enabled");
	shell_print(sh, "transport: in-process FlatBuffer store");
#if defined(CONFIG_CUBS2_CSYN_NATIVE_UDP)
	shell_print(sh, "external transport: native_sim UDP rx=%d tx=%d",
		    CONFIG_CUBS2_CSYN_NATIVE_UDP_RX_PORT,
		    CONFIG_CUBS2_CSYN_NATIVE_UDP_TX_PORT);
#else
	shell_print(sh, "external transport: not compiled");
#endif
	shell_print(sh, "topics: %u", (unsigned int)cubs2_csyn_topic_count());

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_csyn_topic,
	SHELL_CMD(list, NULL, "list csyn topics", cmd_csyn_topic_list),
	SHELL_CMD_ARG(info, &sub_csyn_topic_names, "show topic info: csyn topic info <name>",
		      cmd_csyn_topic_info, 2, 0),
	SHELL_CMD_ARG(echo, &sub_csyn_topic_names,
		      "echo topic once or periodically: csyn topic echo <name> [period_ms]",
		      cmd_csyn_topic_echo, 2, 1),
	SHELL_CMD_ARG(hz, &sub_csyn_topic_names,
		      "measure topic rate: csyn topic hz <name> [window_ms]",
		      cmd_csyn_topic_hz, 2, 1),
	SHELL_CMD(stop, NULL, "stop csyn topic echo/hz", cmd_csyn_topic_stop),
	SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_csyn,
	SHELL_CMD(status, NULL, "show csyn status", cmd_csyn_status),
	SHELL_CMD(topic, &sub_csyn_topic, "csyn topic commands", NULL),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(csyn, &sub_csyn, "csyn FlatBuffer topic commands", NULL);
