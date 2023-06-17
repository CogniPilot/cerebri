/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "actuator_vesc_can.h"
#include <stdio.h>
#include <zephyr/drivers/can.h>

#define CAN_ALIAS(N) STRINGIFY(can ## #N)

canbus_detail_t g_canbus_details[] = {
    { .ready = false },
    { .ready = false },
    { .ready = false },
    { .ready = false },
    { .ready = false },
    { .ready = false }
};

actuator_vesc_can_t g_actuator_vesc_cans[] = {
#if CONFIG_VESC_CAN_NUMBER > 0
    { .bus_alias = CAN_ALIAS(CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0),
        .fd = CONFIG_VESC_CAN_BUS_FD_0,
        .id = CONFIG_VESC_CAN_ID_0,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_0,
        .pole_pair = CONFIG_VESC_POLE_PAIR_0,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 0
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 1
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 2
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 3
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 4
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_0 == 5
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 1
    { .bus_alias = CAN_ALIAS(CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1),
        .fd = CONFIG_VESC_CAN_BUS_FD_1,
        .id = CONFIG_VESC_CAN_ID_1,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_1,
        .pole_pair = CONFIG_VESC_POLE_PAIR_1,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 0
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 1
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 2
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 3
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 4
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_1 == 5
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 2
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2,
        .fd = CONFIG_VESC_CAN_BUS_FD_2,
        .id = CONFIG_VESC_CAN_ID_2,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_2,
        .pole_pair = CONFIG_VESC_POLE_PAIR_2,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_2 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 3
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3,
        .fd = CONFIG_VESC_CAN_BUS_FD_3,
        .id = CONFIG_VESC_CAN_ID_3,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_3,
        .pole_pair = CONFIG_VESC_POLE_PAIR_3,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_3 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 4
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4,
        .fd = CONFIG_VESC_CAN_BUS_FD_4,
        .id = CONFIG_VESC_CAN_ID_4,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_4,
        .pole_pair = CONFIG_VESC_POLE_PAIR_4,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_4 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 5
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5,
        .fd = CONFIG_VESC_CAN_BUS_FD_5,
        .id = CONFIG_VESC_CAN_ID_5,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_5,
        .pole_pair = CONFIG_VESC_POLE_PAIR_5,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_5 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 6
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6,
        .fd = CONFIG_VESC_CAN_BUS_FD_6,
        .id = CONFIG_VESC_CAN_ID_6,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_6,
        .pole_pair = CONFIG_VESC_POLE_PAIR_6,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_6 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
#if CONFIG_VESC_CAN_NUMBER > 7
    { .bus_alias = CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7,
        .fd = CONFIG_VESC_CAN_BUS_FD_7,
        .id = CONFIG_VESC_CAN_ID_7,
        .index = CONFIG_VESC_CAN_ACTUATOR_VEL_IDX_7,
        .pole_pair = CONFIG_VESC_POLE_PAIR_7,
#if CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can0"
        .bus_id = 0,
        .device = DEVICE_DT_GET(DT_ALIAS(can0))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can1"
        .bus_id = 1,
        .device = DEVICE_DT_GET(DT_ALIAS(can1))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can2"
        .bus_id = 2,
        .device = DEVICE_DT_GET(DT_ALIAS(can2))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can3"
        .bus_id = 3,
        .device = DEVICE_DT_GET(DT_ALIAS(can3))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can4"
        .bus_id = 4,
        .device = DEVICE_DT_GET(DT_ALIAS(can4))
#elif CONFIG_VESC_CAN_BUS_ALIAS_CAN_N_7 == "can5"
        .bus_id = 5,
        .device = DEVICE_DT_GET(DT_ALIAS(can5))
#endif
    },
#endif
};

/* vi: ts=4 sw=4 et */
