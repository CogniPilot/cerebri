#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

#include "messages.h"
#include "mixer.h"

#include "casadi/bezier6.h"
#include "casadi/rover.h"
#include "casadi/se2.h"

double gx = 0;
double gy = 0;
double gz = 0;

struct msg_trajectory_t msg_trajectory = {};
struct msg_odometry_t msg_odometry = {};

double auto_thrust = 0;
double auto_steering = 0;
int last_mode_change = -1;
double wheel_radius = 0.0365;
double L = 0.2255;

void auto_mode() {
    uint64_t time_start = msg_trajectory.time_start;
    uint64_t time_stop = msg_trajectory.time_stop;
    int64_t uptime = k_uptime_get()*1e6;
    int64_t T_nsec = time_stop - time_start;
    int64_t t_nsec = uptime - time_start;
    if (t_nsec > T_nsec) {
      // stop
      auto_thrust = 0;
      auto_steering = 0;
      return;
    }
    if (t_nsec < 0) {
      t_nsec = 0;
    }
    double t = t_nsec*1e-9;
    double T = T_nsec*1e-9;
    double x, y, psi, V, delta = 0;
    double e[3] = {}; // e_x, e_y, e_theta
    double PX[6] = {
        msg_trajectory.x[0],
        msg_trajectory.x[1],
        msg_trajectory.x[2],
        msg_trajectory.x[3],
        msg_trajectory.x[4],
        msg_trajectory.x[5]
    };
    double PY[6] = {
        msg_trajectory.y[0],
        msg_trajectory.y[1],
        msg_trajectory.y[2],
        msg_trajectory.y[3],
        msg_trajectory.y[4],
        msg_trajectory.y[5]
    };

    //printf("uptime ms: %ld\n", uptime);
    //printf("start ns: %ld\n", time_start);
    //printf("stop ns: %ld\n", time_stop);
    //printf("t ns: %ld\n", t_nsec);
    //printf("T ns: %ld\n", T_nsec);

    //printf("t: %10.2f\n", t);
    //printf("T: %10.2f\n", T);
    //printf("PX: %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n", PX[0], PX[1], PX[2], PX[3], PX[4], PX[5]);
    //printf("PY: %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n", PY[0], PY[1], PY[2], PY[3], PY[4], PY[5]);
    //printf("L: %10.2f\n", L);

    /* rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,delta) */
    {
        const casadi_real * args[5];
        casadi_real * res[5];
        args[0] = &t;
        args[1] = &T;
        args[2] = PX;
        args[3] = PY;
        args[4] = &L;
        res[0] = &x;
        res[1] = &y;
        res[2] = &psi;
        res[3] = &V;
        res[4] = &delta;
        casadi_int * iw = NULL;
        casadi_real * w = NULL;
        int mem = 0;
        rover(args, res, iw, w, mem);

        printf("x: %10.2f\n", x);
        printf("y: %10.2f\n", y);
        printf("psi: %10.2f\n", psi);
        printf("V: %10.2f\n", V);
        printf("delta: %10.2f\n", delta);
    }

    /* se2_error:(i0[3],i1[3])->(o0[3]) */
    {
        const casadi_real * args[2];
        casadi_real * res[1];
        casadi_int * iw = NULL;
        casadi_real * w = NULL;
        int mem = 0;

        casadi_real i0[3], i1[3];

        // vehicle position
        i0[0] = msg_odometry.x;
        i0[1] = msg_odometry.y;
        i0[2] = 2*atan2(msg_odometry.qz, msg_odometry.qw);

        // reference position
        i1[0] = x;
        i1[1] = y;
        i1[2] = psi;

        // call function
        args[0] = i0;
        args[1] = i1;
        res[0] = e;
        se2_error(args, res, iw, w, mem);
    }

    if (isnan(V)) {
        printf("V is nan\n");
        auto_thrust = 0;
    } else {
        printf("e_x: %15.4f\n", e[0]);
        auto_thrust = 1*e[0] + V/wheel_radius;
    }

    if (isnan(delta)) {
        printf("delta is nan\n");
        auto_steering = 0;
    } else {
        printf("e_y: %15.4f\n", e[1]);
        printf("e_theta: %15.4f\n", e[2]);
        auto_steering = 1*e[1] + 1*e[2] + delta;
    }
}
 
void listener_controller_callback(const struct zbus_channel *chan) {
    
    if (chan == &chan_rc_input) {
        const struct msg_rc_input_t *msg_rc_input = (const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
        struct msg_rc_input_t msg_control = *msg_rc_input;


        // manual mode
        if (msg_rc_input->mode == 1) {
            if (msg_rc_input->mode != last_mode_change) {
                last_mode_change = msg_rc_input->mode;
                printf("Set to manual mode!\n");
            }
            msg_control.thrust = msg_rc_input->thrust;
            msg_control.yaw = msg_rc_input->yaw;

        // auto
        } else if (msg_rc_input->mode == 0) {
            if (msg_rc_input->mode != last_mode_change) {
                last_mode_change = msg_rc_input->mode;
                printf("Set to trajectory follow!\n");
            }
            msg_control.thrust = auto_thrust;
            msg_control.yaw = auto_steering;
            //printf("thrust: %15.2f\n", auto_thrust);
            //printf("yaw: %15.2f\n", auto_steering);
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

        // comptue auto if we have a trajectory
        if (msg_trajectory.time_start != 0) {
            auto_mode();
        }
    }
}

ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);
