/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal host stand-ins for the few Zephyr macros that the shared
 * topic_flatbuffer.c uses, so it can be compiled into the native host build.
 */
#ifndef CUBS2_HOST_COMPAT_ZEPHYR_SYS_UTIL_H_
#define CUBS2_HOST_COMPAT_ZEPHYR_SYS_UTIL_H_

#include <assert.h>

#ifndef BUILD_ASSERT
#define BUILD_ASSERT(EXPR, ...) _Static_assert((EXPR), "" __VA_ARGS__)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#endif /* CUBS2_HOST_COMPAT_ZEPHYR_SYS_UTIL_H_ */
