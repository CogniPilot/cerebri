#ifndef CEREBRI_ZBUS_CHANNELS_H
#define CEREBRI_ZBUS_CHANNELS_H

#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DECLARE(chan_in_cmd_vel);
ZBUS_CHAN_DECLARE(chan_in_joy);
ZBUS_CHAN_DECLARE(chan_in_odometry);

#endif // CEREBRI_ZBUS_CHANNELS_H
