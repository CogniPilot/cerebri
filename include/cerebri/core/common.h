#ifndef CEREBRI_CORE_COMMON_H
#define CEREBRI_CORE_COMMON_H

#include <zephyr/device.h>
const struct device *get_device(const struct device *const dev);

extern const char *banner_brain;
extern const char *banner_name;

#define CASADI_FUNC_ARGS(name)                                                                     \
	casadi_int iw[name##_SZ_IW];                                                               \
	casadi_real w[name##_SZ_W];                                                                \
	const casadi_real *args[name##_SZ_ARG];                                                    \
	casadi_real *res[name##_SZ_RES];                                                           \
	int mem = 0;

#define CASADI_FUNC_CALL(name) name(args, res, iw, w, mem);

void cerebri_set_module_log_level(int16_t module_id, uint32_t level);

#endif // CEREBRI_CORE_COMMON_H
