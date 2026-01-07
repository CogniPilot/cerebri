/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOG_SDCARD_H
#define LOG_SDCARD_H

#include <stdbool.h>

/**
 * @brief Start the log_sdcard reader thread
 * @return 0 on success, negative on error or if already running
 */
int log_sdcard_start(void);

/**
 * @brief Stop the log_sdcard reader thread
 * @return 0 on success, negative if not running
 */
int log_sdcard_stop(void);

/**
 * @brief Check if log_sdcard reader thread is running
 * @return true if running, false otherwise
 */
bool log_sdcard_is_running(void);

/**
 * @brief Start the log_sdcard writer thread
 * @return 0 on success, negative on error or if already running
 */
int log_sdcard_writer_start(void);

/**
 * @brief Stop the log_sdcard writer thread
 * @return 0 on success, negative if not running
 */
int log_sdcard_writer_stop(void);

/**
 * @brief Check if log_sdcard writer thread is running
 * @return true if running, false otherwise
 */
bool log_sdcard_writer_is_running(void);

#endif /* LOG_SDCARD_H */
