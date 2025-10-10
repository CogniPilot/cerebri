#ifndef CEREBRI_LOG_UTILS_H
#define CEREBRI_LOG_UTILS_H

#include <zephyr/logging/log.h>

/* Register a module’s desired runtime log level with the manager */
void cerebri_register_log_level(int16_t module_id, uint32_t level);

/*
 * Convenience macro for nodes:
 * Registers the module with LOG_LEVEL_DBG compiled,
 * and requests runtime filtering at `level`.
 */
#define CEREBRI_NODE_LOG_INIT(name, level)                                                         \
	LOG_MODULE_REGISTER(name, LOG_LEVEL_DBG);                                                  \
	static int name##_log_init(void)                                                           \
	{                                                                                          \
		cerebri_register_log_level(LOG_CURRENT_MODULE_ID(), level);                        \
		return 0;                                                                          \
	}                                                                                          \
	SYS_INIT(name##_log_init, APPLICATION, 98)

#endif // CEREBRI_LOG_UTILS_H
