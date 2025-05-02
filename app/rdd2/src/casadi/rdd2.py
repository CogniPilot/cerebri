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

print('python: ', sys.executable)

# parameters
g = 9.8 # grav accel m/s^2
m = 2.24 # mass of vehicle
#thrust_delta = 0.9*m*g # thrust delta from trim
#thrust_trim = m*g # thrust trim
deg2rad = np.pi/180 # degree to radian

# attitude rate loop
rollpitch_rate_max = 60 # deg/s
yaw_rate_max = 60 # deg/s
rollpitch_max = 30 # deg

# position loop
kp_pos = 1.0 # position proportional gain
kp_vel = 2.0 # velocity proportional gain
#pos_sp_dist_max = 2 # position setpoint max distance
#vel_max = 2.0 # max velocity command
z_integral_max = 0 # 5.0
ki_z = 0.05 # velocity z integral gain

# estimator params
att_w_acc = 0.8
att_w_gyro_bias = 0
param_att_w_mag = 0.2


def derive_control_allocation(
):
    """
    quadrotor control allocation
    """
    n_motors = 4
    l = ca.SX.sym('l')
    Cm = ca.SX.sym('Cm')
    Ct = ca.SX.sym('Ct')
    T = ca.SX.sym('T')
    F_max = ca.SX.sym('F_max') # max thrust of each motor
    M = ca.SX.sym('M', 3)

    T_max = n_motors*F_max
    F_min = 0
    A = ca.vertcat(
        ca.horzcat(1/n_motors, -1/(n_motors*l), -1/(n_motors*l), -1/(n_motors*Cm)),
        ca.horzcat(1/n_motors, 1/(n_motors*l), 1/(n_motors*l), -1/(n_motors*Cm)),
        ca.horzcat(1/n_motors, 1/(n_motors*l), -1/(n_motors*l), 1/(n_motors*Cm)),
        ca.horzcat(1/n_motors, -1/(n_motors*l), 1/(n_motors*l), 1/(n_motors*Cm)))

    T_sat = saturate(T, 0, T_max)
    M_max = l*T_max/2 # max moment when half of motors on and half off
    M_sat = saturatem(M, -M_max*ca.SX.ones(3), M_max*ca.SX.ones(3))

    F_moment = A@ca.vertcat(0, M_sat)  # motor force for moment
    F_thrust = A@ca.vertcat(T_sat, 0, 0, 0)  # motor force for thrust
    F_sum = F_moment + F_thrust

    C1  = F_max - ca.mmax(F_sum) # how much could increase thrust before sat
    C2  = ca.mmin(F_sum) - F_min # how much could decrease thrust before sat

    Fp_thrust = ca.if_else(C1 > 0,
        ca.if_else(C2 > 0,  F_thrust, F_thrust - C2),
        ca.if_else(C2 > 0,  F_thrust + C1, F_max/2*ca.SX.ones(4, 1)))

    Fp_moment = ca.if_else(ca.logic_and(C1 < 0, C2 < 0),
        F_moment*(F_max/2)/ca.mmax(ca.fabs(F_moment)),
        F_moment)

    Fp_sum = saturatem(Fp_moment + Fp_thrust, ca.SX.zeros(n_motors), F_max*ca.SX.ones(n_motors))
    omega = ca.sqrt(Fp_sum/Ct)

    f_alloc = ca.Function("control_allocation",
            [F_max, l, Cm, Ct, T, M],
            [omega],
            ['F_max', 'l', 'Cm', 'Ct', 'T', 'M'],
            ['omega']
            )
    return {
        "f_alloc": f_alloc
    }

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

def derive_joy_acro():
    """
    Acro mode manual input:

    Given joy input, find roll rate and thrust setpoints
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym('thrust_trim')
    thrust_delta = ca.SX.sym('thrust_delta')

    # INPUT VARIABLES
    # -------------------------------
    joy_roll = ca.SX.sym('joy_roll')
    joy_pitch = ca.SX.sym('joy_pitch')
    joy_yaw = ca.SX.sym('joy_yaw')
    joy_thrust = ca.SX.sym('joy_thrust')

    # CALC
    # -------------------------------
    w = ca.vertcat(
        rollpitch_rate_max*deg2rad*joy_roll,
        rollpitch_rate_max*deg2rad*joy_pitch,
        yaw_rate_max*deg2rad*joy_yaw)
    thrust = joy_thrust * thrust_delta + thrust_trim

    # FUNCTION
    # -------------------------------
    f_joy_acro = ca.Function(
       "joy_acro",
       [thrust_trim, thrust_delta, joy_roll, joy_pitch, joy_yaw, joy_thrust],
       [w, thrust],
       ["thrust_trim", "thrust_delta", "joy_roll", "joy_pitch", "joy_yaw", "joy_thrust"],
       ["omega", "thrust"])

    return {
        "joy_acro": f_joy_acro
    }


def derive_joy_auto_level():
    """
    Auto level mode manual input:

    Given joy input, find attitude and thrust set points
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym('thrust_trim')
    thrust_delta = ca.SX.sym('thrust_delta')

    # INPUT VARIABLES
    # -------------------------------
    joy_roll = ca.SX.sym('joy_roll')
    joy_pitch = ca.SX.sym('joy_pitch')
    joy_yaw = ca.SX.sym('joy_yaw')
    joy_thrust = ca.SX.sym('joy_thrust')

    q = SO3Quat.elem(ca.SX.sym('q', 4))

    # CALC
    # -------------------------------
    euler = SO3EulerB321.from_Quat(q)
    yaw = euler.param[0]
    pitch = euler.param[1]
    roll = euler.param[2]

    euler_r = SO3EulerB321.elem(ca.vertcat(
        yaw + yaw_rate_max * deg2rad * joy_yaw,
        rollpitch_max * deg2rad * joy_pitch,
        rollpitch_max * deg2rad * joy_roll))

    q_r = SO3Quat.from_Euler(euler_r)
    thrust = joy_thrust * thrust_delta + thrust_trim

    # FUNCTION
    # -------------------------------
    f_joy_auto_level = ca.Function(
       "joy_auto_level",
       [thrust_trim, thrust_delta, joy_roll, joy_pitch, joy_yaw, joy_thrust, q.param],
       [q_r.param, thrust],
       ["thrust_trim", "thrust_delta", "joy_roll", "joy_pitch", "joy_yaw", "joy_thrust", "q"],
       ["q_r", "thrust"])

    return {
        "joy_auto_level": f_joy_auto_level
    }


def derive_attitude_control():
    """
    Attitude control loop

    Given desired attitude, and attitude, find desired angular velocity
    """

    # INPUT CONSTANTS
    # -------------------------------
    kp = ca.SX.sym('kp', 3)

    # INPUT VARIABLES
    # -------------------------------
    q = ca.SX.sym('q', 4) # actual quat
    q_r = ca.SX.sym('q_r', 4) # quat setpoint

    # CALC
    # -------------------------------
    X = SO3Quat.elem(q)
    X_r = SO3Quat.elem(q_r)

    # Lie algebra
    e = (X.inverse() * X_r).log()  # angular velocity to get to desired att in 1 sec

    omega = kp * e.param # elementwise

    # FUNCTION
    # -------------------------------
    f_attitude_control = ca.Function(
        "attitude_control",
        [kp, q, q_r],
        [omega],
        ["kp", "q", "q_r"],
        ["omega"])

    return {
        "attitude_control": f_attitude_control
    }


def derive_attitude_rate_control():
    """
    Attitude rate control loop

    Given angular velocity , angular vel. set point, and angular velocity error integral,
    find the desired moment and updated angular velocity error integral.
    """

    # CONSTANTS
    # -------------------------------
    kp = ca.SX.sym('kp', 3)
    ki = ca.SX.sym('ki', 3)
    kd = ca.SX.sym('kd', 3)
    i_max = ca.SX.sym('i_max', 3)
    f_cut = ca.SX.sym('f_cut')

    # VARIABLES
    # -------------------------------
    omega = ca.SX.sym('omega', 3)
    omega_r = ca.SX.sym('omega_r', 3)
    i0 = ca.SX.sym('i0', 3)
    e0 = ca.SX.sym('e0', 3)
    de0 = ca.SX.sym('de0', 3)
    dt = ca.SX.sym('dt')

    # CALC
    # -------------------------------

    # actual attitude, expressed as quaternion
    e1 = omega_r - omega
    alpha = 2*ca.pi*dt*f_cut/(2*ca.pi*dt*f_cut + 1)
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
        ["kp", "ki", "kd", "f_cut", "i_max", "omega", "omega_r", "i0", "e0", "de0", "dt"],
        ["M", "i1", "e1", "de1", "alpha"])

    return {
        "attitude_rate_control": f_attitude_rate_control
    }


def derive_position_control():
    """
    Given the position, velocity ,and acceleration set points, find the
    desired attitude and thrust.
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym('thrust_trim')

    # INPUT VARIABLES
    # -------------------------------

    #inputs: position trajectory, velocity trajectory, desired Yaw vel, dt
    #state inputs: position, orientation, velocity, and angular velocity
    #outputs: thrust force, angular errors
    pt_w = ca.SX.sym('pt_w', 3) # desired position world frame
    vt_w = ca.SX.sym('vt_w', 3) # desired velocity world frame
    at_w = ca.SX.sym('at_w', 3) # desired acceleration world frame

    qc_wb = SO3Quat.elem(ca.SX.sym('qc_wb', 4)) # camera orientation
    p_w = ca.SX.sym('p_w', 3) # position in world frame
    v_b = ca.SX.sym('v_b', 3) # velocity in body frame
    q_wb = SO3Quat.elem(ca.SX.sym('q_wb', 4))
    z_i = ca.SX.sym('z_i') # z velocity error integral
    dt = ca.SX.sym('dt') # time step

    # CALC
    # -------------------------------
    R_wb = q_wb.to_Matrix()
    
    v_w = R_wb @ v_b

    e_p = p_w - pt_w
    e_v = v_w - vt_w

    xW = ca.SX([1, 0, 0])
    yW = ca.SX([0, 1, 0])
    zW = ca.SX([0, 0, 1])

    # F = - Kp ep - Kv ev + mg zW + m at_w
    # F = - m * Kp' ep - m * Kv' * ev + mg zW + m at_w
    # Force is normalized by the weight (mg)

    # normalized thrust vector
    p_norm_max = 0.3*m*g
    p_term = -kp_pos * e_p - kp_vel * e_v + m * at_w
    p_norm = ca.norm_2(p_term)
    p_term = ca.if_else(p_norm > p_norm_max, p_norm_max*p_term/p_norm, p_term)

    # throttle integral
    z_i_2 = z_i - e_p[2] * dt
    z_i_2 = saturatem(z_i_2,
            -ca.vertcat(z_integral_max), ca.vertcat(z_integral_max))

    # trim throttle
    T = p_term + thrust_trim * zW + ki_z * z_i * zW

    # thrust
    nT = ca.norm_2(T)
    
    # body up is aligned with thrust
    zB = ca.if_else(nT > 1e-3, T/nT, zW)

    # point y using desired camera direction
    ec = SO3EulerB321.from_Quat(qc_wb)
    yt = ec.param[0]
    xC = ca.vertcat(ca.cos(yt), ca.sin(yt), 0)
    yB = ca.cross(zB, xC)
    nyB = ca.norm_2(yB)
    yB = ca.if_else(nyB > 1e-3, yB/nyB, xW)

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
        [thrust_trim, pt_w, vt_w, at_w, qc_wb.param, p_w, v_b, q_wb.param, z_i, dt], [nT, qr_wb.param, z_i_2], 
        ['thrust_trim', 'pt_w', 'vt_w', 'at_w', 'qc_wb', 'p_w', 'v_b', 'q_wb', 'z_i', 'dt'], 
        ['nT', 'qr_wb', 'z_i_2'])
    
    return {
        "position_control" : f_get_u
    }


def calculate_N(v: SE23LieAlgebraElement, B: ca.SX):
    """
    N term for exp_mixed
    """
    omega = v.Omega
    Omega = omega.to_Matrix()
    OmegaSq = Omega @ Omega
    A = ca.sparsify(ca.horzcat(v.a_b.param, v.v_b.param))
    B = ca.sparsify(B)
    theta = ca.norm_2(omega.param)
    C1 = SERIES["(1 - cos(x))/x^2"](theta)
    C2 = SERIES["(x - sin(x))/x^3"](theta)
    C3 = SERIES["(x^2/2 + cos(x) - 1)/x^4"](theta)
    AB = A @ B
    I3 = ca.SX.eye(3)
    return (
        A
        + AB / 2
        + Omega @ A @ (C1 * np.eye(2) + C2 * B)
        + Omega @ Omega @ A @ (C2 * np.eye(2) + C3 * B)
    )


def exp_mixed(

    X0: SE23LieGroupElement,
    l: SE23LieAlgebraElement,
    r: SE23LieAlgebraElement,
    B: ca.SX,
):
    """
    exp_mixed
    """
    P0 = ca.horzcat(X0.v.param, X0.p.param)
    Pl = calculate_N(l, B)
    Pr = calculate_N(r, -B)
    R0 = X0.R
    Rl = (l).Omega.exp(lie.SO3Quat)
    Rr = (r).Omega.exp(lie.SO3Quat)
    Rr0 = Rr * R0
    R1 = Rr0 * Rl

    I2 = ca.SX.eye(2)
    P1 = Rr0.to_Matrix() @ Pl + (Rr.to_Matrix() @ P0 + Pr) @ (I2 + B)
    return lie.SE23Quat.elem(ca.vertcat(P1[:, 1], P1[:, 0], R1.param))


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
    X1 = exp_mixed(X0, l * dt, r * dt, B * dt)
    # should do q renormalize check here
    f_ins =  ca.Function(
        "strapdown_ins_propagate",
        [X0.param, a_b, omega_b, g, dt],
        [X1.param],
        ["x0", "a_b", "omega_b", "g", "dt"],
        ["x1"],
    )
    eqs = {
        "strapdown_ins_propagate" : f_ins
    }
    return eqs

def derive_attitude_estimator():
    # Define Casadi variables
    q0 = ca.SX.sym("q", 4)
    q = SO3Quat.elem(param=q0)
    mag = ca.SX.sym("mag", 3)
    mag_decl = ca.SX.sym("mag_decl", 1)
    gyro = ca.SX.sym("gyro", 3)
    gyro_used = -gyro
    accel = ca.SX.sym("accel", 3)
    dt = ca.SX.sym("dt", 1)

    # correction angular velocity vector
    correction = ca.SX.zeros(3, 1)

    # Convert vector to world frame and extract xy component
    spin_rate = ca.norm_2(gyro_used)

    mag_earth = q.inverse() @ mag
    mag_err = (
        ca.fmod(ca.atan2(mag_earth[1], mag_earth[0]) - mag_decl + ca.pi, 2 * ca.pi)
        - ca.pi
    )

    # Change gain if spin rate is large
    fifty_dps = 0.873
    gain_mult = ca.if_else(spin_rate > fifty_dps, ca.fmin(spin_rate / fifty_dps, 10), 1)

    # Move magnetometer correction in body frame
    correction += (
        q@ca.vertcat(0,0,mag_err)
        * param_att_w_mag * gain_mult
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
    correction += gyro_used

    # Make the correction
    q1 = q * so3.elem(correction * dt).exp(SO3Quat)
    debug = ca.atan2(mag_earth[1], mag_earth[0]) - mag_decl

    # Return estimator
    f_att_estimator = ca.Function(
        "attitude_estimator",
        [q0, mag, mag_decl, gyro, accel, dt],
        [q1.param, debug],
        ["q", "mag", "mag_decl", "gyro", "accel", "dt"],
        ["q1", "debug"],
    )

    return {"attitude_estimator": f_att_estimator}



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
    parser.add_argument('dest_dir')
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))
    eqs = {}
    eqs.update(derive_attitude_rate_control())
    eqs.update(derive_attitude_control())
    eqs.update(derive_position_control())
    eqs.update(derive_joy_acro())
    eqs.update(derive_joy_auto_level())
    eqs.update(derive_strapdown_ins_propagation())
    eqs.update(derive_control_allocation())
    eqs.update(derive_attitude_estimator())
    eqs.update(derive_position_correction())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="rdd2.c", dest_dir=args.dest_dir)
    print("complete")
