/*
 * Cerebri centralized log-level manager
 *
 * Collects per-module log levels registered via CEREBRI_NODE_LOG_INIT()
 * and applies them automatically once Zephyr logging backends are ready.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log.h>

#ifndef CONFIG_LOG_DOMAIN_ID
#define CONFIG_LOG_DOMAIN_ID 0
#endif

struct cerebri_log_entry {
	int16_t module_id;
	uint32_t level;
};

#define CEREBRI_LOG_MAX_ENTRIES 64

static struct cerebri_log_entry entries[CEREBRI_LOG_MAX_ENTRIES];
static size_t entry_count;
static bool filters_applied;

/* Public API: called by each module to register desired runtime log level */
void cerebri_register_log_level(int16_t module_id, uint32_t level)
{
#if IS_ENABLED(CONFIG_LOG)
	if (entry_count < CEREBRI_LOG_MAX_ENTRIES) {
		entries[entry_count++] = (struct cerebri_log_entry){module_id, level};
	}
#endif
}

/* Internal: apply runtime log filters to all registered modules */
static void cerebri_apply_filters(void)
{
#if IS_ENABLED(CONFIG_LOG_RUNTIME_FILTERING)
	for (size_t e = 0; e < entry_count; e++) {
		for (int i = 0; i < log_backend_count_get(); i++) {
			const struct log_backend *backend = log_backend_get(i);
			if (backend) {
				log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, entries[e].module_id,
					       entries[e].level);
			}
		}
	}
#endif
}

/* Worker to defer until after logging fully initialized */
static void cerebri_log_late_work_fn(struct k_work *work)
{
#if IS_ENABLED(CONFIG_LOG)
	if (!filters_applied) {
		cerebri_apply_filters();
		filters_applied = true;
	}
#endif
}

/* Single delayed worker instance for all modules */
K_WORK_DELAYABLE_DEFINE(cerebri_log_late_work, cerebri_log_late_work_fn);

static int cerebri_log_sys_init(void)
{
#if IS_ENABLED(CONFIG_LOG)
	/* Delay slightly to ensure all backends are initialized */
	k_work_schedule(&cerebri_log_late_work, K_MSEC(100));
#endif
	return 0;
}

/* Run late in boot, after logging backends have started */
SYS_INIT(cerebri_log_sys_init, APPLICATION, 99);
