/*
 * Cerebri log-level manager
 *
 * Registers module log levels during SYS_INIT, then applies them via
 * delayed work after boot completes (when backends are available).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log.h>

#ifndef CONFIG_LOG_DOMAIN_ID
#define CONFIG_LOG_DOMAIN_ID 0
#endif

#define MAX_LOG_MODULES 32

struct log_module_entry {
	int16_t module_id;
	uint32_t level;
};

static struct log_module_entry g_log_modules[MAX_LOG_MODULES];
static int g_log_module_count;
static struct k_work_delayable g_log_filter_work;

static void apply_log_filters_work(struct k_work *work)
{
	ARG_UNUSED(work);
#if IS_ENABLED(CONFIG_LOG_RUNTIME_FILTERING)
	int backend_count = log_backend_count_get();
	for (int m = 0; m < g_log_module_count; m++) {
		for (int i = 0; i < backend_count; i++) {
			const struct log_backend *backend = log_backend_get(i);
			if (backend) {
				log_filter_set(backend, CONFIG_LOG_DOMAIN_ID,
					       g_log_modules[m].module_id, g_log_modules[m].level);
			}
		}
	}
#endif
}

/* Called during SYS_INIT to register desired log level */
void cerebri_register_log_level(int16_t module_id, uint32_t level)
{
	if (g_log_module_count < MAX_LOG_MODULES) {
		g_log_modules[g_log_module_count].module_id = module_id;
		g_log_modules[g_log_module_count].level = level;
		g_log_module_count++;
	}
}

/* Public API: apply runtime log filter for a module (can be called anytime) */
void cerebri_apply_log_filter(int16_t module_id, uint32_t level)
{
#if IS_ENABLED(CONFIG_LOG_RUNTIME_FILTERING)
	for (int i = 0; i < log_backend_count_get(); i++) {
		const struct log_backend *backend = log_backend_get(i);
		if (backend) {
			log_filter_set(backend, CONFIG_LOG_DOMAIN_ID, module_id, level);
		}
	}
#else
	ARG_UNUSED(module_id);
	ARG_UNUSED(level);
#endif
}

static int cerebri_log_init(void)
{
	k_work_init_delayable(&g_log_filter_work, apply_log_filters_work);
	/* Apply filters 10ms after boot - backends should be ready by then */
	k_work_schedule(&g_log_filter_work, K_MSEC(10));
	return 0;
}

SYS_INIT(cerebri_log_init, APPLICATION, 99);
