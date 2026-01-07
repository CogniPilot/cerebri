#ifndef CEREBRI_LOG_UTILS_H
#define CEREBRI_LOG_UTILS_H

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log_backend.h>

/* Register desired log level during SYS_INIT (applied after boot via delayed work) */
void cerebri_register_log_level(int16_t module_id, uint32_t level);

/* Apply runtime log filter immediately for a module (can be called anytime) */
void cerebri_apply_log_filter(int16_t module_id, uint32_t level);

/*
 * Convenience macro for nodes:
 * Registers the module with LOG_LEVEL_DBG compiled in (so DBG can be enabled at runtime),
 * but applies runtime filtering at `level` after boot completes.
 * Use `log enable dbg <module>` shell command to enable debug messages at runtime.
 */
#define CEREBRI_NODE_LOG_INIT(name, level)                                                         \
	LOG_MODULE_REGISTER(name, LOG_LEVEL_DBG);                                                  \
	static int name##_log_init(void)                                                           \
	{                                                                                          \
		cerebri_register_log_level(LOG_CURRENT_MODULE_ID(), level);                        \
		return 0;                                                                          \
	}                                                                                          \
	SYS_INIT(name##_log_init, APPLICATION, 1)

#endif // CEREBRI_LOG_UTILS_H
