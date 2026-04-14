/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define RDD2_TOP_DEFAULT_PERIOD_MS 1000U
#define RDD2_TOP_MAX_THREADS       32U
#define RDD2_TOP_NAME_LEN          31U
#define RDD2_TOP_WATCH_STACK_SIZE  4096U

struct rdd2_top_watch {
	const struct shell *sh;
	uint32_t period_ms;
	bool active;
};

struct rdd2_top_row {
	struct k_thread *thread;
	char name[RDD2_TOP_NAME_LEN + 1U];
	char state[16];
	uint64_t execution_cycles;
	uint64_t display_cycles;
	size_t stack_size;
	size_t stack_used;
	int priority;
	bool current;
};

struct rdd2_top_snapshot {
	struct rdd2_top_row rows[RDD2_TOP_MAX_THREADS];
	size_t count;
	bool truncated;
};

struct rdd2_top_prev_sample {
	struct k_thread *thread;
	uint64_t execution_cycles;
};

static struct rdd2_top_prev_sample g_rdd2_top_prev[RDD2_TOP_MAX_THREADS];
static size_t g_rdd2_top_prev_count;
static uint64_t g_rdd2_top_prev_total_cycles;
static bool g_rdd2_top_prev_valid;
static struct rdd2_top_watch g_rdd2_top_watch;

K_MUTEX_DEFINE(g_rdd2_top_lock);
K_SEM_DEFINE(g_rdd2_top_watch_sem, 0, 1);

static void rdd2_top_watch_thread(void *p0, void *p1, void *p2);

K_THREAD_DEFINE(g_rdd2_top_watch_tid, RDD2_TOP_WATCH_STACK_SIZE, rdd2_top_watch_thread, NULL, NULL,
		NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

static int rdd2_top_find_prev_index(const struct k_thread *thread)
{
	for (size_t i = 0; i < g_rdd2_top_prev_count; ++i) {
		if (g_rdd2_top_prev[i].thread == thread) {
			return (int)i;
		}
	}

	return -1;
}

static uint32_t rdd2_top_percent_x10(uint64_t part, uint64_t total)
{
	if (total == 0U) {
		return 0U;
	}

	return (uint32_t)((part * 1000ULL) / total);
}

static void rdd2_top_snapshot_row(const struct k_thread *cthread, void *user_data)
{
	struct rdd2_top_snapshot *snapshot = user_data;
	struct k_thread *thread = (struct k_thread *)cthread;
	struct rdd2_top_row *row;
	k_thread_runtime_stats_t stats = {0};
	size_t unused = 0U;
	const char *name;
	char state_buf[32];
	int rc;

	if (snapshot->count >= RDD2_TOP_MAX_THREADS) {
		snapshot->truncated = true;
		return;
	}

	row = &snapshot->rows[snapshot->count++];
	memset(row, 0, sizeof(*row));
	row->thread = thread;
	row->priority = thread->base.prio;
	row->current = (thread == k_current_get());
	row->stack_size = thread->stack_info.size;

	name = k_thread_name_get(thread);
	if (name == NULL || name[0] == '\0') {
		snprintk(row->name, sizeof(row->name), "%p", (void *)thread);
	} else {
		snprintk(row->name, sizeof(row->name), "%s", name);
	}

	snprintk(row->state, sizeof(row->state), "%s",
		 k_thread_state_str(thread, state_buf, sizeof(state_buf)));
	if (row->state[0] == '\0') {
		snprintk(row->state, sizeof(row->state), "-");
	}

	rc = k_thread_runtime_stats_get(thread, &stats);
	if (rc == 0) {
		row->execution_cycles = stats.execution_cycles;
	}

	rc = k_thread_stack_space_get(thread, &unused);
	if (rc == 0 && unused <= row->stack_size) {
		row->stack_used = row->stack_size - unused;
	}
}

static int rdd2_top_row_cmp(const void *lhs, const void *rhs)
{
	const struct rdd2_top_row *a = lhs;
	const struct rdd2_top_row *b = rhs;

	if (a->display_cycles < b->display_cycles) {
		return 1;
	}

	if (a->display_cycles > b->display_cycles) {
		return -1;
	}

	if (a->priority < b->priority) {
		return -1;
	}

	if (a->priority > b->priority) {
		return 1;
	}

	return strcmp(a->name, b->name);
}

static void rdd2_top_update_prev(const struct rdd2_top_snapshot *snapshot)
{
	g_rdd2_top_prev_count = snapshot->count;
	if (g_rdd2_top_prev_count > RDD2_TOP_MAX_THREADS) {
		g_rdd2_top_prev_count = RDD2_TOP_MAX_THREADS;
	}

	for (size_t i = 0; i < g_rdd2_top_prev_count; ++i) {
		g_rdd2_top_prev[i].thread = snapshot->rows[i].thread;
		g_rdd2_top_prev[i].execution_cycles = snapshot->rows[i].execution_cycles;
	}
}

static void rdd2_top_watch_stop(void)
{
	unsigned int key = irq_lock();

	g_rdd2_top_watch.active = false;
	g_rdd2_top_watch.sh = NULL;
	g_rdd2_top_watch.period_ms = 0U;

	irq_unlock(key);

	k_sem_give(&g_rdd2_top_watch_sem);
}

static bool rdd2_top_watch_stop_if_shell(const struct shell *sh)
{
	unsigned int key = irq_lock();
	bool stop_watch = g_rdd2_top_watch.active && g_rdd2_top_watch.sh == sh;

	irq_unlock(key);

	if (stop_watch) {
		rdd2_top_watch_stop();
	}

	return stop_watch;
}

static int rdd2_top_watch_start(const struct shell *sh, uint32_t period_ms)
{
	unsigned int key = irq_lock();

	if (g_rdd2_top_watch.active && g_rdd2_top_watch.sh != sh) {
		irq_unlock(key);
		return -EBUSY;
	}

	g_rdd2_top_watch.sh = sh;
	g_rdd2_top_watch.period_ms = period_ms;
	g_rdd2_top_watch.active = true;

	irq_unlock(key);

	k_sem_give(&g_rdd2_top_watch_sem);
	return 0;
}

static int rdd2_top_render(const struct shell *sh, bool watch_mode, uint32_t period_ms)
{
	struct rdd2_top_snapshot snapshot = {0};
	k_thread_runtime_stats_t all_stats = {0};
	uint64_t total_cycles;
	uint64_t delta_total_cycles = 0U;
	int rc;

	k_mutex_lock(&g_rdd2_top_lock, K_FOREVER);

	if (k_thread_runtime_stats_all_get(&all_stats) != 0) {
		rc = -ENOTSUP;
		goto out;
	}

	k_thread_foreach_unlocked(rdd2_top_snapshot_row, &snapshot);

	total_cycles = all_stats.execution_cycles;
	if (g_rdd2_top_prev_valid && total_cycles >= g_rdd2_top_prev_total_cycles) {
		delta_total_cycles = total_cycles - g_rdd2_top_prev_total_cycles;
	}

	for (size_t i = 0; i < snapshot.count; ++i) {
		struct rdd2_top_row *row = &snapshot.rows[i];

		row->display_cycles = row->execution_cycles;
		if (delta_total_cycles > 0U) {
			int prev = rdd2_top_find_prev_index(row->thread);

			if (prev >= 0 &&
			    row->execution_cycles >= g_rdd2_top_prev[prev].execution_cycles) {
				row->display_cycles = row->execution_cycles -
						      g_rdd2_top_prev[prev].execution_cycles;
			}
		}
	}

	qsort(snapshot.rows, snapshot.count, sizeof(snapshot.rows[0]), rdd2_top_row_cmp);

	if (watch_mode) {
		shell_fprintf(sh, SHELL_NORMAL, "\033[2J\033[H");
	}

	shell_print(sh, "kind  prio state        cpu%%  stack%% used/size  name");
	shell_print(
		sh,
		"----  ---- ----------- ----- ------ ---------- -------------------------------");

	for (size_t i = 0; i < snapshot.count; ++i) {
		const struct rdd2_top_row *row = &snapshot.rows[i];
		size_t stack_percent = 0U;
		uint32_t cpu_percent_x10;

		if (row->stack_size > 0U) {
			stack_percent = (row->stack_used * 100U) / row->stack_size;
		}

		cpu_percent_x10 = rdd2_top_percent_x10(row->display_cycles,
						       delta_total_cycles > 0U ? delta_total_cycles
									       : total_cycles);

		shell_print(sh, "%-4s  %4d %-11.11s %3u.%1u%% %5zu%% %4zu/%-5zu  %-31.31s",
			    row->current ? "curr" : "thr", row->priority, row->state,
			    cpu_percent_x10 / 10U, cpu_percent_x10 % 10U, stack_percent,
			    row->stack_used, row->stack_size, row->name);
	}

	if (snapshot.truncated) {
		shell_print(sh, "truncated at %u threads", (unsigned int)RDD2_TOP_MAX_THREADS);
	}

	if (watch_mode) {
		shell_print(sh, "refresh every %u ms; Ctrl-C to stop; cpu%% is %s",
			    (unsigned int)period_ms,
			    delta_total_cycles > 0U ? "since last refresh" : "since boot");
	} else {
		shell_print(sh, "cpu%% is %s",
			    delta_total_cycles > 0U ? "since last sample" : "since boot");
	}

	rdd2_top_update_prev(&snapshot);
	g_rdd2_top_prev_total_cycles = total_cycles;
	g_rdd2_top_prev_valid = true;
	rc = 0;

out:
	k_mutex_unlock(&g_rdd2_top_lock);

	if (rc != 0) {
		shell_error(sh, "thread runtime stats unavailable");
	}

	return rc;
}

static void rdd2_top_watch_thread(void *p0, void *p1, void *p2)
{
	struct rdd2_top_watch watch;

	ARG_UNUSED(p0);
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);

	while (true) {
		(void)k_sem_take(&g_rdd2_top_watch_sem, K_FOREVER);

		while (true) {
			unsigned int key = irq_lock();

			watch = g_rdd2_top_watch;
			irq_unlock(key);

			if (!watch.active || watch.sh == NULL || watch.period_ms == 0U) {
				break;
			}

			(void)rdd2_top_render(watch.sh, true, watch.period_ms);

			if (k_sem_take(&g_rdd2_top_watch_sem, K_MSEC(watch.period_ms)) == 0) {
				continue;
			}
		}
	}
}

static void rdd2_top_ctrl_c_handler(const struct shell *sh, void *user_data)
{
	ARG_UNUSED(user_data);

	(void)rdd2_top_watch_stop_if_shell(sh);
}

static int rdd2_top_shell_init(void)
{
	return shell_ctrl_c_register(rdd2_top_ctrl_c_handler, NULL);
}

static int cmd_top(const struct shell *sh, size_t argc, char **argv)
{
	char *end = NULL;
	unsigned long period_ul = RDD2_TOP_DEFAULT_PERIOD_MS;
	int rc;

	if (argc > 2U) {
		shell_error(sh, "usage: top [period_ms|once|stop]");
		return -EINVAL;
	}

	if (argc == 2U) {
		if (strcmp(argv[1], "once") == 0) {
			(void)rdd2_top_watch_stop_if_shell(sh);
			return rdd2_top_render(sh, false, 0U);
		}

		if (strcmp(argv[1], "stop") == 0) {
			if (!rdd2_top_watch_stop_if_shell(sh)) {
				shell_print(sh, "top watcher not active");
			}
			return 0;
		}

		period_ul = strtoul(argv[1], &end, 10);
		if (end == argv[1] || *end != '\0' || period_ul == 0UL || period_ul > UINT32_MAX) {
			shell_error(sh, "invalid period: %s", argv[1]);
			return -EINVAL;
		}
	}

	rc = rdd2_top_watch_start(sh, (uint32_t)period_ul);
	if (rc == -EBUSY) {
		shell_error(sh, "top watcher already active on another shell");
		return rc;
	}

	return rc;
}

SYS_INIT(rdd2_top_shell_init, APPLICATION, 0);

SHELL_CMD_REGISTER(top, NULL, "watch a compact thread CPU/stack table: top [period_ms|once|stop]",
		   cmd_top);
