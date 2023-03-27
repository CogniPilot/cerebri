#ifndef SYNAPSE_ZBUS_CHANNELS_H
#define SYNAPSE_ZBUS_CHANNELS_H

#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DECLARE(chan_in_cmd_vel);
ZBUS_CHAN_DECLARE(chan_in_joy);
ZBUS_CHAN_DECLARE(chan_in_odometry);
ZBUS_CHAN_DECLARE(chan_in_bezier);
ZBUS_CHAN_DECLARE(chan_sim_clock);
ZBUS_CHAN_DECLARE(chan_out_actuators);

#endif // SYNAPSE_ZBUS_CHANNELS_H
