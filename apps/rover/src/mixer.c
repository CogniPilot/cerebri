 
#include <zephyr/kernel.h>

#include "messages.h"

struct msg_actuators_t mixer(struct msg_rc_input_t * msg_rc_input) {
    struct msg_actuators_t actuators_msg;

    enum control_mode_t mode = msg_rc_input->mode;
    double vt = msg_rc_input->thrust;

    double vy = msg_rc_input->yaw;
  
    bool armed = msg_rc_input->armed;
      


    double mix_thrust = armed ? vt : 0;
    double mix_yaw = armed ? vy : 0;


    double scale0 = 1.0;
    if (mode == MODE_MANUAL) {
        scale0 = (104.8 - -104.8)/2.0;
    }
    double actuator0 = 1.0*mix_thrust*scale0 + -1.0*mix_yaw*scale0;
    if(actuator0 > 104.8) {
        actuator0 = 104.8;
    } else if (actuator0 < -104.8) {
        actuator0 = -104.8;
    }
    actuators_msg.actuator0_value = actuator0;

    double scale1 = 1.0;
    if (mode == MODE_MANUAL) {
        scale1 = (104.8 - -104.8)/2.0;
    }
    double actuator1 =  1.0*mix_thrust*scale1 +  1.0*mix_yaw*scale1;
    if(actuator1 > 104.8) {
        actuator1 = 104.8;
    } else if (actuator1 < -104.8) {
        actuator1 = -104.8;
    }
    actuators_msg.actuator1_value = actuator1;

    double scale2 = 1.0;
    if (mode == MODE_MANUAL) {
        scale2 = (104.8 - -104.8)/2.0;
    }
    double actuator2 =  1.0*mix_thrust*scale2 +  1.0*mix_yaw*scale2;
    if(actuator2 > 104.8) {
        actuator2 = 104.8;
    } else if (actuator2 < -104.8) {
        actuator2 = -104.8;
    }
    actuators_msg.actuator2_value = actuator2;

    double scale3 = 1.0;
    if (mode == MODE_MANUAL) {
        scale3 = (104.8 - -104.8)/2.0;
    }
    double actuator3 =  1.0*mix_thrust*scale3 +  -1.0*mix_yaw*scale3;
    if(actuator3 > 104.8) {
        actuator3 = 104.8;
    } else if (actuator3 < -104.8) {
        actuator3 = -104.8;
    }
    actuators_msg.actuator3_value = actuator3;


    return actuators_msg;
}
