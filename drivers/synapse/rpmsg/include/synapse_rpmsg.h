/*
 * Copyright (c) 2025 CogniPilot Foundation
 * Copyright 2025 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stddef.h>

void rpmsg_synapse_send(uint8_t *buf, size_t size);
size_t rpmsg_synapse_receive(uint8_t *buf, size_t size);
void rpmsg_rpdev_wait();
