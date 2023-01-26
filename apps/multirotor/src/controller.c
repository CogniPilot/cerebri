#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

float gx = 0;
float gy = 0;
float gz = 0;void listener_controller_callback(const struct zbus_channel *chan) {
    
    
    if (chan == &chan_rc_input) {
        const struct msg_rc_input_t *msg_rc_input = (const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
        struct msg_actuators_t actuators_msg;
        double vt = msg_rc_input->thrust;

        double vy = msg_rc_input->yaw;
  
        bool armed = msg_rc_input->armed;

        double mix_thrust = armed ? vt : 0;
        double mix_yaw = armed ? vy : 0;
    
        double scale0 = (0.3 - -0.3) / 2.0;
        double actuator0 = 0*mix_thrust*scale0 + 1*mix_yaw*scale0;
        if(actuator0 > 0.3) {
            actuator0 = 0.3;
        } else if (actuator0 < -0.3) {
            actuator0 = -0.3;
        }
        actuators_msg.actuator0_value = actuator0;
    
        double scale1 = (50 - -50) / 2.0;
        double actuator1 =  1*mix_thrust*scale1 +  0*mix_yaw*scale1;
        if(actuator1 > 50) {
            actuator1 = 50;
        } else if (actuator1 < -50) {
            actuator1 = -50;
        }
        actuators_msg.actuator1_value = actuator1;
    

        zbus_chan_pub(&chan_actuators, &actuators_msg, K_NO_WAIT);

    } else if (chan == &chan_gyroscope) {
        const struct msg_gyroscope_t * msg_gyroscope = (const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
        gx = msg_gyroscope->x;
        gy = msg_gyroscope->y;
        gz = msg_gyroscope->z;
    }
}
ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);
