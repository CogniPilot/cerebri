#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>

#include "channels.h"

void listener_estimator_callback(const struct zbus_channel *chan) {
    if (chan == &chan_accelerometer) {
        //const struct msg_accelerometer_t *msg = (const struct msg_accelerometer_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_gyroscope) {
        //const struct msg_gyroscope_t *msg = (const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_magnetometer) {
        //const struct msg_magnetometer_t *msg = (const struct msg_magnetometer_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_altimeter) {
        //const struct msg_altimeter_t *msg = (const struct msg_altimeter_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_navsat) {
        //const struct msg_navsat_t *msg = (const struct msg_navsat_t *)zbus_chan_const_msg(chan);
    }
}
ZBUS_LISTENER_DEFINE(listener_estimator, listener_estimator_callback);
