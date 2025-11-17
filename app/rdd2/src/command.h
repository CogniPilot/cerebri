/*
 * Copyright CogniPilot Foundation 2024
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <cerebri/core/casadi.h>
#include <cerebri/core/log_utils.h>

#include "app/rdd2/casadi/bezier.h"
#include "app/rdd2/casadi/rdd2.h"
#include "lib/core/common/casadi/common.h"

#include "input_mapping.h"

struct context {
	struct zros_node node;
	synapse_pb_Input input;
	synapse_pb_Vector3 angular_velocity_ff, angular_velocity_sp, force_sp, accel_sp, moment_ff,
		velocity_sp, position_sp;
	synapse_pb_Quaternion attitude_sp, orientation_sp;
	synapse_pb_BezierTrajectory bezier_trajectory;
	synapse_pb_ClockOffset clock_offset;
	synapse_pb_Status status;
	synapse_pb_Status last_status;
	synapse_pb_Odometry odometry_estimator;
	synapse_pb_Twist cmd_vel;
	struct zros_sub sub_bezier_trajectory_ethernet, sub_status, sub_input_ethernet,
		sub_input_sbus, sub_odometry_estimator, sub_cmd_vel_ethernet,
		sub_clock_offset_ethernet;
	struct zros_pub pub_attitude_sp, pub_angular_velocity_ff, pub_angular_velocity_sp,
		pub_force_sp, pub_accel_sp, pub_moment_ff, pub_velocity_sp, pub_orientation_sp,
		pub_position_sp, pub_input;
	struct k_sem running;
	size_t stack_size;
	k_thread_stack_t *stack_area;
	struct k_thread thread_data;
	double psi_sp;        // yaw setpoint for input_velocity function
	double input_aetr[4]; // input: aileron, elevator, throttle, rudder
	double q[4];          // estimated attitude quaternion
	double dt;            // time delta between input updates
	double thrust_trim;   // trim throttle
	double thrust_delta;  // throttle scaling
};

// mode handlers
void rdd2_mode_attitude_rate(struct context *ctx);
void rdd2_mode_attitude(struct context *ctx);
void rdd2_mode_velocity(struct context *ctx);
void rdd2_mode_bezier(struct context *ctx);

// vi: ts=4 sw=4 et
