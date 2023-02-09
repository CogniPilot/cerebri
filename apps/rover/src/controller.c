#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

#include "messages.h"
#include "mixer.h"

#include "casadi/bezier6.h"
#include "casadi/rover.h"
#include "casadi/se2.h"

// data
struct msg_gyroscope_t msg_gyroscope = {};
struct msg_trajectory_t msg_trajectory = {};
struct msg_odometry_t msg_odometry = {};
struct msg_rc_input_t msg_rc_input = {};

// gains
double gain_heading = 0.5;
double gain_cross_track = 0.4;
double gain_along_track = 1.0;

// parameters
double wheel_radius = 0.0365;
double L = 0.2255;

// local variables
double auto_thrust = 0;
double auto_steering = 0;
char * flight_mode_name[2] = {"manual", "auto"};

// listens to zbus channels
void listener_controller_callback(const struct zbus_channel *chan) {
    if (chan == &chan_rc_input) {
        msg_rc_input = *(const struct msg_rc_input_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_gyroscope) {
        msg_gyroscope = *(const struct msg_gyroscope_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_trajectory) {
        msg_trajectory = *(const struct msg_trajectory_t *)zbus_chan_const_msg(chan);
    } else if (chan == &chan_external_odometry) {
        msg_odometry = *(const struct msg_odometry_t *)zbus_chan_const_msg(chan);
    }
}

// computes thrust/steering in auto mode
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
        auto_thrust = gain_along_track*e[0] + V/wheel_radius;
    }

    if (isnan(delta)) {
        printf("delta is nan\n");
        auto_steering = 0;
    } else {
        auto_steering = gain_cross_track*e[1] + gain_heading*e[2] + delta;
    }
}

void control_entry_point(void *, void *, void *) {
    enum flight_mode_t mode = FLIGHT_MODE_MANUAL;
    struct msg_rc_input_t msg_control = {};

    while (true) {

        msg_control.timestamp = msg_rc_input.timestamp;
        msg_control.armed = msg_rc_input.armed;
        msg_control.thrust = 0;
        msg_control.yaw = 0;

        // notify on mode change
        if (msg_rc_input.mode != mode) {
            mode = msg_rc_input.mode;
            printf("mode changed to %s!\n", flight_mode_name[mode]);
        }

        // manual mode
        if (mode == FLIGHT_MODE_MANUAL) {
            msg_control.timestamp = msg_rc_input.timestamp;
            msg_control.thrust = msg_rc_input.thrust;
            msg_control.yaw = msg_rc_input.yaw;

        // auto
        } else if (mode == FLIGHT_MODE_AUTO) {
            // comptue auto if we have a trajectory
            if (msg_trajectory.time_start != 0) {
                auto_mode();
                msg_control.thrust = auto_thrust;
                msg_control.yaw = auto_steering;
            }
        }

        // send data to motors
        struct msg_actuators_t msg_actuators = mixer(&msg_control);
        msg_actuators.timestamp = msg_control.timestamp;

        zbus_chan_pub(&chan_actuators, &msg_actuators, K_NO_WAIT);

        // sleep to set control rate at 50 Hz
        k_usleep(1e6/50);
    }
}

ZBUS_LISTENER_DEFINE(listener_controller, listener_controller_callback);

K_THREAD_DEFINE(control_thread, 500,
                control_entry_point, NULL, NULL, NULL,
                5, 0, 0);
