/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ubxlib.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

uDeviceCfg_t gDeviceCfg;

uNetworkCfgGnss_t gNetworkCfg = {
    .type = U_NETWORK_TYPE_GNSS
};

LOG_MODULE_REGISTER(ubx_gnss, CONFIG_CEREBRI_SENSE_UBX_GNSS_LOG_LEVEL);

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 6

typedef struct context {
    const struct device* device[CONFIG_CEREBRI_SENSE_MAG_COUNT];
    struct zros_node node;
    struct zros_pub pub;
    synapse_msgs_NavSatFix data;
    int32_t gMeasurementPeriodMs;
    bool running;
    bool isAlive;
} context_t;

static context_t g_ctx = {
    .device = {},
    .node = {},
    .pub = {},
    .data = {
        .has_header = true,
        .header = {
            .frame_id = "wgs84",
            .has_stamp = true,
            .seq = 0,
            .stamp = synapse_msgs_Time_init_default,
        },
        .altitude = 0,
        .latitude = 0,
        .longitude = 0,
        .position_covariance = {},
        .position_covariance_count = 0,
        .position_covariance_type = 0,
        .status = { .service = 0, .status = 0 } },
    .gMeasurementPeriodMs = 100,
    .running = false,
    .isAlive = false,
};

void publish_gnss_data(uDeviceHandle_t devHandle,
    int32_t errorCode,
    const uLocation_t* pLocation)
{
    context_t* ctx = &g_ctx;
    ctx->isAlive = true;

    if (errorCode == 0) {
        synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_default;

        ctx->data.latitude = pLocation->latitudeX1e7 / 1e7;
        ctx->data.longitude = pLocation->longitudeX1e7 / 1e7;
        ctx->data.altitude = pLocation->altitudeMillimetres / 1e3;
        stamp_header(&ctx->data.header, k_uptime_ticks());
        ctx->data.header.seq++;

        // TODO Covariance
        zros_pub_update(&ctx->pub);
        LOG_DBG("lat %f long %f\n", msg.latitude, msg.longitude);
    } else if (errorCode == U_ERROR_COMMON_TIMEOUT) {
        // LOG_ERR("Tiemout error");
    } else {
        LOG_ERR("GNSS error %i", errorCode);
        g_ctx.running = false;
    }
}

void sense_ubx_gnss_entry_point(context_t* ctx)
{
    zros_node_init(&ctx->node, "sense_ubx_gnss");
    zros_pub_init(&ctx->pub, &ctx->node, &topic_nav_sat_fix, &ctx->data);
    int32_t errorCode;

    // Remove the line below if you want the log printouts from ubxlib
    uPortLogOff();
    // Initiate ubxlib
    // And the U-blox GNSS module
    uDeviceHandle_t deviceHandle;

    while (true) {
        uPortInit();
        uDeviceInit();

        uDeviceGetDefaults(U_DEVICE_TYPE_GNSS, &gDeviceCfg);

        gDeviceCfg.transportCfg.cfgUart.uart = 0;
        gDeviceCfg.transportCfg.cfgUart.baudRate = 38400;
        gDeviceCfg.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M8;

        errorCode = uDeviceOpen(&gDeviceCfg, &deviceHandle);
        LOG_DBG("Opened the GNSS device %i\n", errorCode);
        if (errorCode == 0) {
            // Bring up the GNSS
            if (uNetworkInterfaceUp(deviceHandle, U_NETWORK_TYPE_GNSS, &gNetworkCfg) == 0) {
                LOG_DBG("Starting continuous location.\n");
                uLocationGetContinuousStart(deviceHandle,
                    ctx->gMeasurementPeriodMs,
                    U_LOCATION_TYPE_GNSS,
                    NULL, NULL, publish_gnss_data);

                ctx->running = true;

                while (ctx->running) {
                    uPortTaskBlock(1000);
                    /* If cb didnt set isAlive to true hw is malfunctioning reset */
                    if (!ctx->isAlive) {
                        break;
                    }
                    /* Challange cb to set alive to ture */
                    ctx->isAlive = false;
                }

                uLocationGetStop(deviceHandle);

                LOG_DBG("Taking down GNSS...\n");
                uNetworkInterfaceDown(deviceHandle, U_NETWORK_TYPE_GNSS);
                uDeviceClose(deviceHandle, false);
            } else {
                LOG_ERR("Unable to bring up GNSS!\n");
            }

        } else {
            LOG_ERR("Failed to initiate the module: %d", errorCode);
            uPortTaskBlock(1000);
        }

        uDeviceDeinit();
        uPortDeinit();
    }
}

K_THREAD_DEFINE(ubx_gnss, MY_STACK_SIZE,
    sense_ubx_gnss_entry_point, &g_ctx, NULL, NULL,
    MY_PRIORITY, 0, 0);

// vi: ts=4 sw=4 et
