#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

float gx = 0;
float gy = 0;
float gz = 0;

void listener_controller_callback(const struct zbus_channel *chan) {
    if (chan == &chan_rc_input) {
        const struct msg_rc_input_t *msg_rc_input = (const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
        double vt = msg_rc_input->thrust;
        double vr = msg_rc_input->roll;
        double vp = msg_rc_input->pitch;
        double vy = msg_rc_input->yaw;
        bool armed = msg_rc_input->armed;
        double kd = 0.05;

        double thrust_trim = 0.76;
        double mix_thrust = armed ? vt + thrust_trim : 0;
        double mix_roll = armed ? vr*0.1 - kd*gx : 0;
        double mix_pitch = armed ? vp*0.1 - kd*gy : 0;
        double mix_yaw = armed ? vy*0.1 - kd*gz : 0;

        struct msg_actuators_t actuators_msg;
        actuators_msg.ch0 = mix_thrust - mix_roll - mix_pitch - mix_yaw; // motor 0 (front right)
        actuators_msg.ch1 = mix_thrust + mix_roll + mix_pitch - mix_yaw; // motor 1 (rear left)
        actuators_msg.ch2 = mix_thrust + mix_roll - mix_pitch + mix_yaw; // motor 2 (front left)
        actuators_msg.ch3 = mix_thrust - mix_roll + mix_pitch + mix_yaw; // motor 3 (rear right)
        zbus_chan_pub(&chan_actuators, &actuators_msg, K_NO_WAIT);

    } else if (chan == &chan_gyroscope) {
        const struct msg_gyroscope_t * msg_gyroscope = (const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
        gx = msg_gyroscope->x;
        gy = msg_gyroscope->y;
        gz = msg_gyroscope->z;
    }
}
ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);

