/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>

typedef struct actuator_vesc_can_t {
    const char* bus_alias;
    bool fd;
    uint8_t id;
    uint8_t index;
    uint8_t pole_pair;
    uint8_t bus_id;
    const struct device* device;
} actuator_vesc_can_t;

typedef struct canbus_detail_t {
    bool ready;
} canbus_detail_t;
