import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import SO3Quat, SO3EulerB321, so3
from cyecca.lie.group_se23 import (
    SE23Quat,
    se23,
    SE23LieGroupElement,
    SE23LieAlgebraElement,
)
from cyecca.symbolic import SERIES

# parameters
g = 9.8  # grav accel m/s^2
m = 2.24  # mass of vehicle
# thrust_delta = 0.9*m*g # thrust delta from trim
# thrust_trim = m*g # thrust trim
deg2rad = np.pi / 180  # degree to radian

# attitude rate loop
rollpitch_rate_max = 60  # deg/s
yaw_rate_max = 60  # deg/s
rollpitch_max = 30  # deg

# position loop
kp_pos = 1.0  # position proportional gain
kp_vel = 2.0  # velocity proportional gain
# pos_sp_dist_max = 2 # position setpoint max distance
# vel_max = 2.0 # max velocity command
z_integral_max = 0  # 5.0
ki_z = 0.05  # velocity z integral gain

# estimator params
att_w_acc = 0.2
att_w_gyro_bias = 0.1
param_att_w_mag = 0.2


def derive_control_allocation():
    """
    quadrotor control allocation
    """
    n_motors = 4
    l = ca.SX.sym("l")
    Cm = ca.SX.sym("Cm")
    Ct = ca.SX.sym("Ct")
    T = ca.SX.sym("T")
    F_max = ca.SX.sym("F_max")  # max thrust of each motor
    M = ca.SX.sym("M", 3)

    T_max = n_motors * F_max
    F_min = 0
    A = ca.vertcat(
        ca.horzcat(
            1 / n_motors, -1 / (n_motors * l), -1 / (n_motors * l), -1 / (n_motors * Cm)
        ),
        ca.horzcat(
            1 / n_motors, 1 / (n_motors * l), 1 / (n_motors * l), -1 / (n_motors * Cm)
        ),
        ca.horzcat(
            1 / n_motors, 1 / (n_motors * l), -1 / (n_motors * l), 1 / (n_motors * Cm)
        ),
        ca.horzcat(
            1 / n_motors, -1 / (n_motors * l), 1 / (n_motors * l), 1 / (n_motors * Cm)
        ),
    )

    T_sat = saturate(T, 0, T_max)
    M_max = l * T_max / 2  # max moment when half of motors on and half off
    M_sat = saturatem(M, -M_max * ca.SX.ones(3), M_max * ca.SX.ones(3))

    F_moment = A @ ca.vertcat(0, M_sat)  # motor force for moment
    F_thrust = A @ ca.vertcat(T_sat, 0, 0, 0)  # motor force for thrust
    F_sum = F_moment + F_thrust

    saturation_logic = True

    if saturation_logic:
        C1 = F_max - ca.mmax(F_sum)  # how much could increase thrust before sat
        C2 = ca.mmin(F_sum) - F_min  # how much could decrease thrust before sat

        Fp_thrust = ca.if_else(
            C1 > 0,
            ca.if_else(C2 > 0, F_thrust, F_thrust - C2),
            ca.if_else(C2 > 0, F_thrust + C1, F_max / 2 * ca.SX.ones(4, 1)),
        )

        largest_moment = ca.mmax(ca.fabs(F_moment))

        # if one motor is at full throttle, and another motor is off, and a moment is commanded
        # that is not zeros, then rescale the moment thrust components to max thrust / 2
        Fp_moment = ca.if_else(
            ca.logic_and(ca.logic_and(C1 < 0, C2 < 0), largest_moment > 1e-5),
            (F_max / 2) * F_moment / largest_moment,
            F_moment,
        )

        Fp_sum = saturatem(
            Fp_moment + Fp_thrust, ca.SX.zeros(n_motors), F_max * ca.SX.ones(n_motors)
        )
    else:
        Fp_sum = F_sum

    omega = ca.sqrt(Fp_sum / Ct)

    f_alloc = ca.Function(
        "control_allocation",
        [F_max, l, Cm, Ct, T, M],
        [omega, Fp_sum, F_moment, F_thrust, M_sat],
        ["F_max", "l", "Cm", "Ct", "T", "M"],
        ["omega", "Fp_sum", "F_moment", "F_thrust", "M_sat"],
    )
    return {"f_alloc": f_alloc}


def saturatem(x, x_min, x_max):
    """
    saturate a matrix
    """
    y = ca.SX.zeros(x.shape[0])
    for i in range(x.shape[0]):
        y[i] = saturate(x[i], x_min[i], x_max[i])
    return y


def saturate(x, x_min, x_max):
    """
    saturate
    """
    return ca.if_else(x > x_max, x_max, ca.if_else(x < x_min, x_min, x))


def derive_velocity_control():

    # INPUT VARIABLES
    # -------------------------------
    dt = ca.SX.sym("dt")
    psi_sp = ca.SX.sym("psi_sp")
    pw_sp = ca.SX.sym("pw_sp", 3)
    pw = ca.SX.sym("pw", 3)
    vb = ca.SX.sym("vb", 3)
    psi_vel_sp = ca.SX.sym("psi_vel_sp")
    reset_position = ca.SX.sym("reset_position")

    # CALC
    # -------------------------------
    psi_sp1 = psi_sp + psi_vel_sp * dt
    psi_sp1 = ca.remainder(psi_sp1, 2 * ca.pi)

    cos_yaw = ca.cos(psi_sp1)
    sin_yaw = ca.sin(psi_sp1)

    vw_sp = ca.vertcat(
        vb[0] * cos_yaw - vb[1] * sin_yaw, vb[0] * sin_yaw + vb[1] * cos_yaw, vb[2]
    )

    pw_sp1 = ca.if_else(reset_position, pw, pw_sp + vw_sp * dt)

    e = pw_sp1 - pw
    e_max = 2
    e_norm = ca.norm_2(e)
    e = ca.if_else(e_norm > e_max, 2 * e / e_norm, e)

    q_sp = SO3Quat.from_Euler(SO3EulerB321.elem(ca.vertcat(psi_sp1, 0, 0))).param

    pw_sp1 = pw + e

    aw_sp = ca.vertcat(0, 0, 0)

    # FUNCTION
    # -------------------------------
    f_velocity_control = ca.Function(
        "velocity_control",
        [
            dt,
            psi_sp,
            pw_sp,
            pw,
            vb,
            psi_vel_sp,
            reset_position,
        ],
        [psi_sp1, pw_sp1, vw_sp, aw_sp, q_sp],
        [
            "dt",
            "psi_sp",
            "pw_sp",
            "pw",
            "vb",
            "psi_vel_sp",
            "reset_position",
        ],
        ["psi_sp1", "pw_sp1", "vw_sp", "aw_sp", "q_sp"],
    )
    return {"velocity_control": f_velocity_control}


def derive_input_acro():
    """
    Acro mode manual input:

    Given input, find roll rate and thrust setpoints
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym("thrust_trim")
    thrust_delta = ca.SX.sym("thrust_delta")

    # INPUT VARIABLES
    # -------------------------------
    input_aetr = ca.SX.sym("input_aetr", 4)

    # CALC
    # -------------------------------
    w = ca.vertcat(
        rollpitch_rate_max * deg2rad * input_aetr[0],
        rollpitch_rate_max * deg2rad * input_aetr[1],
        yaw_rate_max * deg2rad * input_aetr[3],
    )
    thrust = input_aetr[2] * thrust_delta + thrust_trim

    # FUNCTION
    # -------------------------------
    f_input_acro = ca.Function(
        "input_acro",
        [thrust_trim, thrust_delta, input_aetr],
        [w, thrust],
        [
            "thrust_trim",
            "thrust_delta",
            "input_aetr",
        ],
        ["omega", "thrust"],
    )

    return {"input_acro": f_input_acro}


def derive_input_auto_level():
    """
    Auto level mode manual input:

    Given manual input, find attitude and thrust set points
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym("thrust_trim")
    thrust_delta = ca.SX.sym("thrust_delta")

    # INPUT VARIABLES
    # -------------------------------
    input_aetr = ca.SX.sym("input_aetr", 4)  # aileron, elevator, thrust, rudder
    q = SO3Quat.elem(ca.SX.sym("q", 4))

    # CALC
    # -------------------------------
    euler = SO3EulerB321.from_Quat(q)
    yaw = euler.param[0]
    pitch = euler.param[1]
    roll = euler.param[2]

    euler_r = SO3EulerB321.elem(
        ca.vertcat(
            yaw + yaw_rate_max * deg2rad * input_aetr[3],
            rollpitch_max * deg2rad * input_aetr[1],
            rollpitch_max * deg2rad * input_aetr[0],
        )
    )

    q_r = SO3Quat.from_Euler(euler_r)
    thrust = input_aetr[2] * thrust_delta + thrust_trim

    # FUNCTION
    # -------------------------------
    f_input_auto_level = ca.Function(
        "input_auto_level",
        [thrust_trim, thrust_delta, input_aetr, q.param],
        [q_r.param, thrust],
        [
            "thrust_trim",
            "thrust_delta",
            "input_aetr",  # aileron, elevator, thrust, rudder
            "q",
        ],
        ["q_r", "thrust"],
    )

    return {"input_auto_level": f_input_auto_level}


def derive_input_velocity():
    # INPUT VARIABLES
    # -------------------------------
    input_aetr = ca.SX.sym("input_aetr", 4)

    # CALC
    # -------------------------------
    psi_vel_sp = 60 * deg2rad * input_aetr[3]
    vb = ca.vertcat(2 * input_aetr[1], -2 * input_aetr[0], input_aetr[2])

    # FUNCTION
    # -------------------------------
    f_input_velocity = ca.Function(
        "input_velocity",
        [
            input_aetr,
        ],
        [vb, psi_vel_sp],
        [
            "input_aetr",
        ],
        ["vb", "psi_vel_sp"],
    )
    return {"input_velocity": f_input_velocity}


def derive_attitude_control():
    """
    Attitude control loop

    Given desired attitude, and attitude, find desired angular velocity
    """

    # INPUT CONSTANTS
    # -------------------------------
    kp = ca.SX.sym("kp", 3)

    # INPUT VARIABLES
    # -------------------------------
    q = ca.SX.sym("q", 4)  # actual quat
    q_r = ca.SX.sym("q_r", 4)  # quat setpoint

    # CALC
    # -------------------------------
    X = SO3Quat.elem(q)
    X_r = SO3Quat.elem(q_r)

    # Lie algebra
    e = (X.inverse() * X_r).log()  # angular velocity to get to desired att in 1 sec

    omega = kp * e.param  # elementwise

    # FUNCTION
    # -------------------------------
    f_attitude_control = ca.Function(
        "attitude_control", [kp, q, q_r], [omega], ["kp", "q", "q_r"], ["omega"]
    )

    return {"attitude_control": f_attitude_control}


def derive_attitude_rate_control():
    """
    Attitude rate control loop

    Given angular velocity , angular vel. set point, and angular velocity error integral,
    find the desired moment and updated angular velocity error integral.
    """

    # CONSTANTS
    # -------------------------------
    kp = ca.SX.sym("kp", 3)
    ki = ca.SX.sym("ki", 3)
    kd = ca.SX.sym("kd", 3)
    i_max = ca.SX.sym("i_max", 3)
    f_cut = ca.SX.sym("f_cut")

    # VARIABLES
    # -------------------------------
    omega = ca.SX.sym("omega", 3)
    omega_r = ca.SX.sym("omega_r", 3)
    i0 = ca.SX.sym("i0", 3)
    e0 = ca.SX.sym("e0", 3)
    de0 = ca.SX.sym("de0", 3)
    dt = ca.SX.sym("dt")

    # CALC
    # -------------------------------

    # actual attitude, expressed as quaternion
    e1 = omega_r - omega
    alpha = 2 * ca.pi * dt * f_cut / (2 * ca.pi * dt * f_cut + 1)
    de1 = alpha * ((e1 - e0) / dt) + (1 - alpha) * de0
    # first order deriv approx, with low pass filter

    # integral action helps balance distrubance moments (e.g. center of gravity offset)
    i1 = saturatem(i0 + e1 * dt, -i_max, i_max)

    M = kp * e1 + ki * i1 + kd * de1

    # FUNCTION
    # -------------------------------
    f_attitude_rate_control = ca.Function(
        "attitude_rate_control",
        [kp, ki, kd, f_cut, i_max, omega, omega_r, i0, e0, de0, dt],
        [M, i1, e1, de1, alpha],
        [
            "kp",
            "ki",
            "kd",
            "f_cut",
            "i_max",
            "omega",
            "omega_r",
            "i0",
            "e0",
            "de0",
            "dt",
        ],
        ["M", "i1", "e1", "de1", "alpha"],
    )

    return {"attitude_rate_control": f_attitude_rate_control}


def derive_position_control():
    """
    Given the position, velocity ,and acceleration set points, find the
    desired attitude and thrust.
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym("thrust_trim")

    # INPUT VARIABLES
    # -------------------------------

    # inputs: position trajectory, velocity trajectory, desired Yaw vel, dt
    # state inputs: position, orientation, velocity, and angular velocity
    # outputs: thrust force, angular errors
    pt_w = ca.SX.sym("pt_w", 3)  # desired position world frame
    vt_w = ca.SX.sym("vt_w", 3)  # desired velocity world frame
    at_w = ca.SX.sym("at_w", 3)  # desired acceleration world frame

    qc_wb = SO3Quat.elem(ca.SX.sym("qc_wb", 4))  # camera orientation
    p_w = ca.SX.sym("p_w", 3)  # position in world frame
    v_w = ca.SX.sym("v_w", 3)  # velocity in world frame
    z_i = ca.SX.sym("z_i")  # z velocity error integral
    dt = ca.SX.sym("dt")  # time step

    # CALC
    # -------------------------------
    e_p = p_w - pt_w
    e_v = v_w - vt_w

    xW = ca.SX([1, 0, 0])
    yW = ca.SX([0, 1, 0])
    zW = ca.SX([0, 0, 1])

    # F = - Kp ep - Kv ev + mg zW + m at_w
    # F = - m * Kp' ep - m * Kv' * ev + mg zW + m at_w
    # Force is normalized by the weight (mg)

    # normalized thrust vector
    p_norm_max = 0.3 * m * g
    p_term = -kp_pos * e_p - kp_vel * e_v + m * at_w
    p_norm = ca.norm_2(p_term)
    p_term = ca.if_else(p_norm > p_norm_max, p_norm_max * p_term / p_norm, p_term)

    # throttle integral
    z_i_2 = z_i - e_p[2] * dt
    z_i_2 = saturatem(z_i_2, -ca.vertcat(z_integral_max), ca.vertcat(z_integral_max))

    # trim throttle
    T = p_term + thrust_trim * zW + ki_z * z_i * zW

    # thrust
    nT = ca.norm_2(T)

    # body up is aligned with thrust
    zB = ca.if_else(nT > 1e-3, T / nT, zW)

    # point y using desired camera direction
    ec = SO3EulerB321.from_Quat(qc_wb)
    yt = ec.param[0]
    xC = ca.vertcat(ca.cos(yt), ca.sin(yt), 0)
    yB = ca.cross(zB, xC)
    nyB = ca.norm_2(yB)
    yB = ca.if_else(nyB > 1e-3, yB / nyB, xW)

    # point x using cross product of unit vectors
    xB = ca.cross(yB, zB)

    # desired attitude matrix
    Rd_wb = ca.horzcat(xB, yB, zB)
    # [bx_wx by_wx bz_wx]
    # [bx_wy by_wy bz_wy]
    # [bx_wz by_wz bz_wz]

    # deisred euler angles
    # note using euler angles as set point is not problematic
    # using Lie group approach for control
    qr_wb = SO3Quat.from_Matrix(Rd_wb)

    # FUNCTION
    # -------------------------------
    f_get_u = ca.Function(
        "position_control",
        [thrust_trim, pt_w, vt_w, at_w, qc_wb.param, p_w, v_w, z_i, dt],
        [nT, qr_wb.param, z_i_2],
        ["thrust_trim", "pt_w", "vt_w", "at_w", "qc_wb", "p_w", "v_w", "z_i", "dt"],
        ["nT", "qr_wb", "z_i_2"],
    )

    return {"position_control": f_get_u}


def derive_common():
    q = SO3Quat.elem(ca.SX.sym("q", 4))
    vw0 = ca.SX.sym("vw0", 3)
    vb1 = ca.SX.sym("vb1", 3)
    vb0 = q.inverse() @ vw0
    vw1 = q @ vb1
    f_rotate_vector_w_to_b = ca.Function(
        "rotate_vector_w_to_b", [q.param, vw0], [vb0], ["q", "v_w"], ["v_b"]
    )
    f_rotate_vector_b_to_w = ca.Function(
        "rotate_vector_b_to_w", [q.param, vb1], [vw1], ["q", "v_b"], ["v_w"]
    )
    return {
        "rotate_vector_w_to_b": f_rotate_vector_w_to_b,
        "rotate_vector_b_to_w": f_rotate_vector_b_to_w,
    }


def derive_strapdown_ins_propagation():
    """
    INS strapdown propagation
    """
    dt = ca.SX.sym("dt")
    X0 = lie.SE23Quat.elem(ca.SX.sym("X0", 10))
    a_b = ca.SX.sym("a_b", 3)
    g = ca.SX.sym("g")
    omega_b = ca.SX.sym("omega_b", 3)
    l = lie.se23.elem(ca.vertcat(0, 0, 0, a_b, omega_b))
    r = lie.se23.elem(ca.vertcat(0, 0, 0, 0, 0, -g, 0, 0, 0))
    B = ca.sparsify(ca.SX([[0, 1], [0, 0]]))
    X1 = lie.SE23Quat.exp_mixed(X0, l * dt, r * dt, B * dt)
    # should do q renormalize check here
    f_ins = ca.Function(
        "strapdown_ins_propagate",
        [X0.param, a_b, omega_b, g, dt],
        [X1.param],
        ["x0", "a_b", "omega_b", "g", "dt"],
        ["x1"],
    )
    eqs = {"strapdown_ins_propagate": f_ins}
    return eqs


def derive_position_correction():
    ## Initilaizing measurments
    z = ca.SX.sym("gps", 3)
    dt = ca.SX.sym("dt", 1)
    P = ca.SX.sym("P", 6, 6)

    # Initialize state
    est_x = ca.SX.sym("est", 10)  # [x,y,z,u,v,w,q0,q1,q2,q3]
    x0 = est_x[0:6]  # [x,y,z,u,v,w]

    # Define the state transition matrix (A)
    A = ca.SX.eye(6)
    A[0:3, 3:6] = np.eye(3) * dt  # The velocity elements multiply by dt

    ## TODO: may need to pass Q and R throught the casadi function
    Q = np.eye(6) * 1e-5  # Process noise (uncertainty in system model)
    R = np.eye(3) * 1e-2  # Measurement noise (uncertainty in sensors)

    # Measurement matrix
    H = ca.horzcat(ca.SX.eye(3), ca.SX.zeros(3, 3))

    # extrapolate uncertainty
    P_s = A @ P @ A.T + Q

    ## Measurment Update
    ## vel is a basic integral given acceleration values. need to figure out how to get v0
    y = H @ P_s @ H.T + R

    # Update Kalman Gain
    K = P_s @ H.T @ ca.inv(y)

    # Update estimate w/ measurment
    x_new = x0 + K @ (z - H @ x0)

    # Update the measurement uncertainty
    P_new = (np.eye(6) - (K @ H)) @ P_s

    # Return to have attitude updated
    x_new = ca.vertcat(x_new, ca.SX.zeros(4))

    f_pos_estimator = ca.Function(
        "position_correction",
        [est_x, z, dt, P],
        [x_new, P_new],
        ["est_x", "gps", "dt", "P"],
        ["x_new", "P_new"],
    )
    return {"position_correction": f_pos_estimator}


def derive_attitude_estimator():
    # Define Casadi variables
    q0 = ca.SX.sym("q", 4)
    q = SO3Quat.elem(param=q0)
    mag = ca.SX.sym("mag", 3)
    mag_decl = ca.SX.sym("mag_decl", 1)
    gyro = ca.SX.sym("gyro", 3)
    accel = ca.SX.sym("accel", 3)
    dt = ca.SX.sym("dt", 1)

    # Convert magnetometer to quat
    mag1 = SO3Quat.elem(ca.vertcat(0, mag))

    # correction angular velocity vector
    correction = ca.SX.zeros(3, 1)

    # Convert vector to world frame and extract xy component
    spin_rate = ca.norm_2(gyro)
    mag_earth = (q.inverse() * mag1 * q).param[1:]

    mag_err = (
        ca.fmod(ca.atan2(mag_earth[1], mag_earth[0] - mag_decl) + ca.pi, 2 * ca.pi)
        - ca.pi
    )

    # Change gain if spin rate is large
    fifty_dps = 0.873
    gain_mult = ca.if_else(spin_rate > fifty_dps, ca.fmin(spin_rate / fifty_dps, 10), 1)

    # Move magnetometer correction in body frame
    correction += (
        (q.inverse() * SO3Quat.elem(ca.vertcat(0, 0, 0, mag_err)) * q).param[1:]
        * param_att_w_mag
        * gain_mult
    )

    # Correction from accelerometer
    accel_norm_sq = ca.norm_2(accel) ** 2

    # Correct accelerometer only if g between
    higher_lim_check = ca.if_else(accel_norm_sq < ((g * 1.1) ** 2), 1, 0)
    lower_lim_check = ca.if_else(accel_norm_sq > ((g * 0.9) ** 2), 1, 0)

    # Correct gravity as z
    correction += (
        lower_lim_check
        * higher_lim_check
        * ca.cross(np.array([[0], [0], [-1]]), accel / ca.norm_2(accel))
        * att_w_acc
    )

    ## TODO add gyro bias stuff

    # Add gyro to correction
    correction += gyro

    # Make the correction
    q1 = q * so3.elem(correction * dt).exp(SO3Quat)

    # Return estimator
    f_att_estimator = ca.Function(
        "attitude_estimator",
        [q0, mag, mag_decl, gyro, accel, dt],
        [q1.param],
        ["q", "mag", "mag_decl", "gyro", "accel", "dt"],
        ["q1"],
    )

    return {"attitude_estimator": f_att_estimator}


def generate_code(eqs: dict, filename, dest_dir: str, **kwargs):
    """
    Generate C Code from python CasADi functions.
    """
    dest_dir = Path(dest_dir)
    dest_dir.mkdir(exist_ok=True)
    p = {
        "verbose": True,
        "mex": False,
        "cpp": False,
        "main": False,
        "with_header": True,
        "with_mem": False,
        "with_export": False,
        "with_import": False,
        "include_math": True,
        "avoid_stack": True,
    }
    for k, v in kwargs.items():
        assert k in p.keys()
        p[k] = v

    gen = ca.CodeGenerator(filename, p)
    for name, eq in eqs.items():
        gen.add(eq)
    gen.generate(str(dest_dir) + os.sep)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dest_dir")
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))
    eqs = {}
    eqs.update(derive_attitude_rate_control())
    eqs.update(derive_attitude_control())
    eqs.update(derive_velocity_control())
    eqs.update(derive_position_control())
    eqs.update(derive_input_acro())
    eqs.update(derive_input_auto_level())
    eqs.update(derive_input_velocity())
    eqs.update(derive_strapdown_ins_propagation())
    eqs.update(derive_control_allocation())
    eqs.update(derive_common())
    eqs.update(derive_attitude_estimator())
    eqs.update(derive_position_correction())

    for name, eq in eqs.items():
        print("eq: ", name)

    generate_code(eqs, filename="rdd2.c", dest_dir=args.dest_dir)
    print("complete")
