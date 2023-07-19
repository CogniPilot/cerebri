/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ubxlib.h"
#include <stdio.h>
#include <string.h>
#include <synapse/zbus/channels.h>
#include <time.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

uDeviceCfg_t gDeviceCfg;

uNetworkCfgGnss_t gNetworkCfg = {
    .type = U_NETWORK_TYPE_GNSS
};

LOG_MODULE_REGISTER(ubx_gnss, CONFIG_UBX_GNSS_LOG_LEVEL);

#define MY_STACK_SIZE 8192
#define MY_PRIORITY 6

void publish_gnss_data_zbus(uLocation_t* location)
{
    synapse_msgs_NavSatFix nav_sat_fix;

    nav_sat_fix.latitude = location->longitudeX1e7 / 1e6;
    nav_sat_fix.longitude = location->longitudeX1e7 / 1e6;
    nav_sat_fix.altitude = location->altitudeMillimetres / 1e3;

    // TODO Covariance

    zbus_chan_pub(&chan_out_nav_sat_fix, &nav_sat_fix, K_NO_WAIT);
}

void sense_ubx_gnss_entry_point()
{
    int32_t errorCode;

    // Remove the line below if you want the log printouts from ubxlib
    uPortLogOff();
    // Initiate ubxlib
    uPortInit();
    uDeviceInit();
    // And the U-blox GNSS module
    uDeviceHandle_t deviceHandle;
    uDeviceGetDefaults(U_DEVICE_TYPE_GNSS, &gDeviceCfg);

    gDeviceCfg.transportCfg.cfgUart.uart = 0;
    gDeviceCfg.transportCfg.cfgUart.baudRate = 38400;
    gDeviceCfg.deviceCfg.cfgGnss.moduleType = U_GNSS_MODULE_TYPE_M8;

    errorCode = uDeviceOpen(&gDeviceCfg, &deviceHandle);
    LOG_DBG("Opened the GNSS device %i\n", errorCode);
    if (errorCode == 0) {
        // Bring up the GNSS
        errorCode = uNetworkInterfaceUp(deviceHandle, U_NETWORK_TYPE_GNSS, &gNetworkCfg);
        if (errorCode == 0) {
            while (true) {
                uLocation_t location;
                errorCode = uLocationGet(deviceHandle, U_LOCATION_TYPE_GNSS,
                    NULL, NULL, &location, NULL);
                if (errorCode == 0) {
                    publish_gnss_data_zbus(&location);
                } else if (errorCode == U_ERROR_COMMON_TIMEOUT) {
                    LOG_ERR("Location Timeout\n");
                } else {
                    LOG_ERR("Failed to get position: %d\n", errorCode);
                }
            }
        } else {
            LOG_ERR("Failed to bring up the GNSS: %d", errorCode);
        }
        uDeviceClose(deviceHandle, true);
    } else {
        LOG_ERR("Failed to initiate the module: %d", errorCode);
    }
}

K_THREAD_DEFINE(ubx_gnss_thread, MY_STACK_SIZE,
    sense_ubx_gnss_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);
