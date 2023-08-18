/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdio.h>

#include <synapse/zbus/channels.h>

#include "casadi/rover.h"
#include "parameters.h"

LOG_MODULE_REGISTER(control_ackermann, CONFIG_CONTROL_ACKERMANN_LOG_LEVEL);

#define MY_STACK_SIZE 3072
#define MY_PRIORITY 4

enum control_mode_t {
    MODE_INIT = 0,
    MODE_MANUAL = 1,
    MODE_AUTO = 2,
    MODE_CMD_VEL = 3,
};
typedef enum control_mode_t control_mode_t;
static char* g_mode_name[4] = { "init", "manual", "auto", "cmd_vel" };
static int32_t g_seq = 0;

control_mode_t g_mode = MODE_INIT;
bool g_armed = false;
static synapse_msgs_Odometry g_pose = synapse_msgs_Odometry_init_default;
static synapse_msgs_Twist g_cmd_vel = synapse_msgs_Twist_init_default;
static synapse_msgs_Joy g_joy = synapse_msgs_Joy_init_default;
static synapse_msgs_BezierTrajectory g_bezier_trajectory = synapse_msgs_BezierTrajectory_init_default;
static synapse_msgs_Time g_clock_offset = synapse_msgs_Time_init_default;
static synapse_msgs_BatteryState g_battery_state = synapse_msgs_BatteryState_init_default;

static void handle_joy()
{
    // arming
    if (g_joy.buttons[7] == 1 && !g_armed) {
        if (g_mode == MODE_INIT) {
            LOG_WRN("cannot arm until mode selected");
            return;
        }
        LOG_INF("armed in mode: %s", g_mode_name[g_mode]);
        LOG_INF("battery voltage: %f", g_battery_state.voltage);
        g_armed = true;
    } else if (g_joy.buttons[6] == 1 && g_armed) {
        LOG_INF("disarmed");
        g_armed = false;
        g_mode = MODE_INIT;
    }

    // handle modes
    control_mode_t prev_mode = g_mode;
    if (g_joy.buttons[0] == 1) {
        g_mode = MODE_MANUAL;
    } else if (g_joy.buttons[1] == 1) {
        g_mode = MODE_AUTO;
    } else if (g_joy.buttons[2] == 1) {
        g_mode = MODE_CMD_VEL;
    }

    // notify on mode change
    if (g_mode != prev_mode) {
        LOG_INF("mode changed to: %s", g_mode_name[g_mode]);
    }
}

static void listener_control_ackermann_callback(const struct zbus_channel* chan)
{
    if (chan == &chan_in_joy) {
        g_joy = *(synapse_msgs_Joy*)(chan->message);
        handle_joy();
    } else if (chan == &chan_in_odometry) {
        g_pose = *(synapse_msgs_Odometry*)(chan->message);
    } else if (g_mode == MODE_CMD_VEL && chan == &chan_in_cmd_vel) {
        g_cmd_vel = *(synapse_msgs_Twist*)(chan->message);
    } else if (chan == &chan_in_bezier_trajectory) {
        g_bezier_trajectory = *(synapse_msgs_BezierTrajectory*)(chan->message);
    } else if (chan == &chan_in_clock_offset) {
        g_clock_offset = *(synapse_msgs_Time*)(chan->message);
    } else if (chan == &chan_out_battery_state) {
        g_battery_state = *(synapse_msgs_BatteryState*)(chan->message);
    }
}

ZBUS_LISTENER_DEFINE(listener_control_ackermann, listener_control_ackermann_callback);

// computes rc_input from V, omega
void mixer()
{

    // given cmd_vel, compute actuators
    synapse_msgs_Actuators msg = synapse_msgs_Actuators_init_default;

    double turn_angle = 0;
    double omega_fwd = 0;

    /* ackermann_steering:(L,omega,V)->(delta) */
    if (g_mode == MODE_MANUAL) {
        turn_angle = max_turn_angle * g_joy.axes[3];
        omega_fwd = 0.25 * max_velocity * g_joy.axes[1] / wheel_radius;
    } else {
        double V = g_cmd_vel.linear.x;
        double omega = g_cmd_vel.angular.z;
        casadi_int* iw = NULL;
        casadi_real* w = NULL;
        int mem = 0;
        double delta = 0;
        const casadi_real* args[3];
        casadi_real* res[1];
        args[0] = &wheel_base;
        args[1] = &omega;
        args[2] = &V;
        res[0] = &delta;
        ackermann_steering(args, res, iw, w, mem);
        omega_fwd = V / wheel_radius;
        if (fabs(V) > 0.01) {
            turn_angle = delta;
        }
    }
    if (!g_armed) {
        omega_fwd = 0;
        turn_angle = 0;
    }

    msg.has_header = true;
    int64_t uptime_ticks = k_uptime_ticks();
    int64_t sec = uptime_ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    int32_t nanosec = (uptime_ticks - sec * CONFIG_SYS_CLOCK_TICKS_PER_SEC) * 1e9 / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
    msg.header.seq = g_seq++;
    msg.header.has_stamp = true;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    strncpy(msg.header.frame_id, "map", sizeof(msg.header.frame_id) - 1);

    msg.position_count = 1;
    msg.velocity_count = 1;
    msg.normalized_count = 2;
    msg.position[0] = turn_angle;
    msg.velocity[0] = omega_fwd;
#ifdef CONFIG_BUGGY3_MOTOR_ENB_REQUIRED
    msg.normalized[0] = -1;
#endif
    zbus_chan_pub(&chan_out_actuators, &msg, K_NO_WAIT);
}

void stop()
{
    // stop
    g_cmd_vel.linear.x = 0;
    g_cmd_vel.angular.z = 0;
}

// computes thrust/steering in auto mode
void auto_mode()
{
    // goal -> given position goal, find cmd_vel
    uint64_t time_start_nsec = g_bezier_trajectory.time_start;
    uint64_t time_stop_nsec = time_start_nsec;

    // get current time
    uint64_t time_nsec = k_uptime_get() * 1e6 + g_clock_offset.sec * 1e9 + g_clock_offset.nanosec;

    if (time_nsec < time_start_nsec) {
        LOG_INF("time current: %" PRIu64
                " ns < time start: %" PRIu64
                "  ns, time out of range of trajectory\n",
            time_nsec, time_start_nsec);
        stop();
        return;
    }

    // find current trajectory index, time_start, and time_stop
    int curve_index = 0;
    while (true) {

        // check if time handled by current trajectory
        if (time_nsec < g_bezier_trajectory.curves[curve_index].time_stop) {
            time_stop_nsec = g_bezier_trajectory.curves[curve_index].time_stop;
            if (curve_index > 0) {
                time_start_nsec = g_bezier_trajectory.curves[curve_index - 1].time_stop;
            }
            break;
        }

        // next index
        curve_index++;

        // check if index exceeds bounds
        if (curve_index >= g_bezier_trajectory.curves_count) {
            stop();
            return;
        }
    }

    double T = (time_stop_nsec - time_start_nsec) * 1e-9;
    double t = (time_nsec - time_start_nsec) * 1e-9;
    double x, y, psi, V, omega = 0;
    double e[3] = {}; // e_x, e_y, e_theta

    double PX[6], PY[6];
    for (int i = 0; i < 6; i++) {
        PX[i] = g_bezier_trajectory.curves[curve_index].x[i];
        PY[i] = g_bezier_trajectory.curves[curve_index].y[i];
    }

    // casadi mem args
    casadi_int* iw = NULL;
    casadi_real* w = NULL;
    int mem = 0;

    /* bezier6_rover:(t,T,PX[1x6],PY[1x6],L)->(x,y,psi,V,omega) */
    {
        const casadi_real* args[5];
        casadi_real* res[5];
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
        const casadi_real* args[2];
        casadi_real* res[1];

        double p[3], r[3];

        // vehicle position
        p[0] = g_pose.pose.pose.position.x;
        p[1] = g_pose.pose.pose.position.y;
        p[2] = 2 * atan2(g_pose.pose.pose.orientation.z, g_pose.pose.pose.orientation.w);

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

    // compute twist
    g_cmd_vel.linear.x = V + gain_along_track * e[0];
    g_cmd_vel.angular.z = omega + gain_cross_track * e[1] + gain_heading * e[2];
}

void control_ackermann_entry_point(void* p1, void* p2, void* p3)
{

    while (true) {
        if (g_mode == MODE_AUTO) {
            auto_mode();
        }

        mixer();

        // sleep to set control rate at 50 Hz
        k_usleep(1e6 / 50);
    }
}

K_THREAD_DEFINE(control_ackermann, MY_STACK_SIZE,
    control_ackermann_entry_point, NULL, NULL, NULL,
    MY_PRIORITY, 0, 0);

/* vi: ts=4 sw=4 et */
