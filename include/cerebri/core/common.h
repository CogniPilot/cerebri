#ifndef CEREBRI_CORE_COMMON_H
#define CEREBRI_CORE_COMMON_H

#include <zephyr/device.h>
const struct device* get_device(const struct device* const dev);

extern const char* banner_brain;
extern const char* banner_name;

#endif // CEREBRI_CORE_COMMON_H
