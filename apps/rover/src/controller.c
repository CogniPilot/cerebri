#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

#include "messages.h"
#include "mixer.h"

float gx = 0;
float gy = 0;
float gz = 0;
 
void listener_controller_callback(const struct zbus_channel *chan) {
    
    if (chan == &chan_rc_input) {
        const struct msg_rc_input_t *msg_rc_input = (const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
        struct msg_actuators_t actuators_msg = mixer(msg_rc_input);
        zbus_chan_pub(&chan_actuators, &actuators_msg, K_NO_WAIT);
    } else if (chan == &chan_gyroscope) {
        const struct msg_gyroscope_t * msg_gyroscope = (const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
        gx = msg_gyroscope->x;
        gy = msg_gyroscope->y;
        gz = msg_gyroscope->z;
    } else if (chan == &chan_trajectory) {
        const struct msg_trajectory_t * msg_trajectory = (const struct msg_trajectory_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_external_odometry) {
        const struct msg_odometry_t * msg_odom = (const struct msg_odometry_t *)zbus_chan_const_msg(chan);
    }
}

ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);
