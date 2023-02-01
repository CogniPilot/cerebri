#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

#include "messages.h"
#include "mixer.h"

double gx = 0;
double gy = 0;
double gz = 0;

struct msg_trajectory_t msg_trajectory;
struct msg_odometry_t msg_odometry;

double dx = 0;
double dy = 0;
int last_mode_change;
 
void listener_controller_callback(const struct zbus_channel *chan) {
    
    if (chan == &chan_rc_input) {
        const struct msg_rc_input_t *msg_rc_input = (const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
        struct msg_rc_input_t msg_control = *msg_rc_input;

        if (msg_rc_input->mode == 1) {
            if (msg_rc_input->mode != last_mode_change) {
                last_mode_change = msg_rc_input->mode;
                printf("Set to manual mode!\n");
            }
            msg_control.thrust = msg_rc_input->thrust;
            msg_control.yaw = msg_rc_input->yaw;
        } else if (msg_rc_input->mode == 0) {
            if (msg_rc_input->mode != last_mode_change) {
                last_mode_change = msg_rc_input->mode;
                printf("Set to trajectory follow!\n");
            }
            msg_control.thrust = dx;
            msg_control.yaw = dy;
        }

	    struct msg_actuators_t actuators_msg = mixer(&msg_control);
        zbus_chan_pub(&chan_actuators, &actuators_msg, K_NO_WAIT);
    } else if (chan == &chan_gyroscope) {
        const struct msg_gyroscope_t * msg_gyroscope = (const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
        gx = msg_gyroscope->x;
        gy = msg_gyroscope->y;
        gz = msg_gyroscope->z;
    } else if (chan == &chan_trajectory) {
        msg_trajectory = *(const struct msg_trajectory_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_external_odometry) {
        msg_odometry = *(const struct msg_odometry_t *)zbus_chan_const_msg(chan);
        double time_start = msg_trajectory.time_start;
        int64_t uptime = k_uptime_get();
        double time_now = uptime/1.0e3;
        double t = time_now - time_start;
        double x = 0;
        double y = 0;
        double t_pow = 1;
        for (int i=6; i>0; i--) {
            x += msg_trajectory.x[i]*t_pow;
            y += msg_trajectory.y[i]*t_pow;
            t_pow = t_pow*t;
        }
        dx = x - msg_odometry.x;
        dy = y - msg_odometry.y;
    }
}

ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);

