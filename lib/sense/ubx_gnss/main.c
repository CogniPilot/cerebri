/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ubxlib.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <cerebri/synapse/zbus/channels.h>

uDeviceCfg_t gDeviceCfg;

uNetworkCfgGnss_t gNetworkCfg = {
    .type = U_NETWORK_TYPE_GNSS
};

LOG_MODULE_REGISTER(ubx_gnss, CONFIG_CEREBRI_SENSE_UBX_GNSS_LOG_LEVEL);

#define MY_STACK_SIZE 2420
#define MY_PRIORITY 6

static int32_t gMeasurementPeriodMs = 100;
static bool running = false;
static bool isAlive = false;
static int32_t g_seq = 0;

void publish_gnss_data_zbus(uDeviceHandle_t devHandle,
    int32_t errorCode,
    const uLocation_t* pLocation)
{
    isAlive = true;

    if (errorCode == 0) {
        synapse_msgs_NavSatFix msg = synapse_msgs_NavSatFix_init_default;

        msg.has_header = true;
        strncpy(msg.header.frame_id, "map", sizeof(msg.header.frame_id) - 1);
        msg.header.has_stamp = true;
        msg.latitude = pLocation->latitudeX1e7 / 1e7;
        msg.longitude = pLocation->longitudeX1e7 / 1e7;
        msg.altitude = pLocation->altitudeMillimetres / 1e3;
        stamp_header(&msg.header, k_uptime_ticks());
        msg.header.seq = g_seq++;

        // TODO Covariance

        zbus_chan_pub(&chan_out_nav_sat_fix, &msg, K_NO_WAIT);
        LOG_DBG("lat %f long %f\n", msg.latitude, msg.longitude);
    } else if (errorCode == U_ERROR_COMMON_TIMEOUT) {
        // LOG_ERR("Tiemout error");
    } else {
        LOG_ERR("GNSS error %i", errorCode);
        running = false;
    }
}

void sense_ubx_gnss_entry_point()
{
    int32_t errorCode;
    g_seq = 0;

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
                    gMeasurementPeriodMs,
                    U_LOCATION_TYPE_GNSS,
                    NULL, NULL, publish_gnss_data_zbus);

                running = true;

                while (running) {
                    uPortTaskBlock(1000);
                    /* If cb didnt set isAlive to true hw is malfunctioning reset */
                    if (!isAlive) {
                        break;
                    }
                    /* Challange cb to set alive to ture */
                    isAlive = false;
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
    sense_ubx_gnss_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);
