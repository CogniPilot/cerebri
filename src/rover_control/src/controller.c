#include <zephyr/kernel.h>
#include <math.h>

#include "channels.h"
#include <stdio.h>

#include "messages.h"
#include "mixer.h"
#include "parameters.h"

#include "casadi/rover.h"

// data
struct msg_gyroscope_t msg_gyroscope = {};
struct msg_trajectory_t msg_trajectory = {};
struct msg_odometry_t msg_odometry = {};
struct msg_rc_input_t msg_rc_input = {};

// local variables
double auto_thrust = 0;
double auto_steering = 0;
char * mode_name[2] = {"manual", "auto"};

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
    double x, y, psi, V, omega = 0;
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

    // casadi mem args
    casadi_int * iw = NULL;
    casadi_real * w = NULL;
    int mem = 0;

    /* bezier6_rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,omega) */
    {
        const casadi_real * args[5];
        casadi_real * res[5];
        args[0] = &t;
        args[1] = &T;
        args[2] = PX;
        args[3] = PY;
        args[4] = &wheel_base;
        res[0] = &x;
        res[1] = &y;
        res[2] = &psi;
        res[3] = &V;
        res[4] = &omega;
        bezier6_rover(args, res, iw, w, mem);
    }

    /* se2_error:(p[3],r[3])->(error[3]) */
    {
        const casadi_real * args[2];
        casadi_real * res[1];

        double p[3], r[3];

        // vehicle position
        p[0] = msg_odometry.x;
        p[1] = msg_odometry.y;
        p[2] = 2*atan2(msg_odometry.qz, msg_odometry.qw);

        // reference position
        r[0] = x;
        r[1] = y;
        r[2] = psi;

        // call function
        args[0] = p;
        args[1] = r;
        res[0] = e;
        se2_error(args, res, iw, w, mem);
    }

#ifdef STEERING_ACKERMANN

    /* ackermann_steering:(L,omega,V)->(delta) */
    {
        double delta = 0;
        const casadi_real * args[3];
        casadi_real * res[1];
        args[0] = &wheel_base;
        args[1] = &omega;
        args[2] = &V;
        res[0] = &delta;
        ackermann_steering(args, res, iw, w, mem);
        auto_thrust = gain_along_track*e[0] + V/wheel_radius;
        auto_steering = gain_cross_track*e[1] + gain_heading*e[2] + delta;
    }
#endif // STEERING_ACKERMANN

#ifdef STEERING_DIFFERENTIAL
    /* differential_steering:(L,omega,w)->(Vw) */
    {
        double Vw = 0;
        const casadi_real * args[3];
        casadi_real * res[1];
        args[0] = &wheel_base;
        args[1] = &omega;
        args[2] = &wheel_separation;
        res[0] = &Vw;
        differential_steering(args, res, iw, w, mem);
        auto_thrust = gain_along_track*e[0] + V/wheel_radius;
        auto_steering = gain_cross_track*e[1] + gain_heading*e[2] + Vw/wheel_radius;
    }
#endif // STEERING_DIFFERENTIAL


}

void control_entry_point(void *, void *, void *) {
    enum control_mode_t mode = MODE_MANUAL;
    struct msg_rc_input_t msg_control = {};

    while (true) {

        msg_control.timestamp = msg_rc_input.timestamp;
        msg_control.armed = msg_rc_input.armed;
        msg_control.thrust = 0;
        msg_control.yaw = 0;

        // notify on mode change
        if (msg_rc_input.mode != mode) {
            mode = msg_rc_input.mode;
            printf("mode changed to %s!\n", mode_name[mode]);
        }

        // manual mode
        if (mode == MODE_MANUAL) {
            msg_control.timestamp = msg_rc_input.timestamp;
            msg_control.thrust = msg_rc_input.thrust;
            msg_control.yaw = msg_rc_input.yaw;
            msg_control.mode = mode;

        // auto
        } else if (mode == MODE_AUTO) {
            // comptue auto if we have a trajectory
            if (msg_trajectory.time_start != 0) {
                auto_mode();
                msg_control.thrust = auto_thrust;
                msg_control.yaw = auto_steering;
                msg_control.mode = mode;
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
                -1, 0, 0);
