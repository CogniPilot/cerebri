import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import SO3Quat, SO3EulerB321
from cyecca.lie.group_se23 import SE23Quat, se23, SE23LieGroupElement, SE23LieAlgebraElement
from cyecca.symbolic import SERIES

print('python: ', sys.executable)

# parameters
g = 9.8 # grav accel m/s^2
m = 2.24 # mass of vehicle
#thrust_delta = 0.9*m*g # thrust delta from trim
#thrust_trim = m*g # thrust trim
deg2rad = np.pi/180 # degree to radian

# attitude rate loop
rollpitch_rate_max = 30 # deg/s
yaw_rate_max = 60 # deg/s

#done kp_rollpitch_rate = 0.3
#done ki_rollpitch_rate = 0.05
#done rollpitch_rate_integral_max = 1.0

# done kp_yaw_rate = 0.3
# done ki_yaw_rate = 0.05
# done yaw_rate_integral_max = 1.0

# attitude loop
rollpitch_max = 20 # deg
#kp_rollpitch = 2
#kp_yaw = 1

# position loop
kp_pos = 0.5 # position proportional gain
kp_vel = 2.0 # velocity proportional gain
#pos_sp_dist_max = 2 # position setpoint max distance
#vel_max = 2.0 # max velocity command
z_integral_max = 0 # 5.0
ki_z = 0.05 # velocity z integral gain

def derive_control_allocation(
):
    """
    quadrotor control allocation
    """
    l = ca.SX.sym('l')
    Cm = ca.SX.sym('Cm')
    Ct = ca.SX.sym('Ct')
    T = ca.SX.sym('T')
    Mx = ca.SX.sym('Mx')
    My = ca.SX.sym('My')
    Mz = ca.SX.sym('Mz')
    A = ca.vertcat(
        ca.horzcat(1/4, -1/(4*l), -1/(4*l), -1/(4*Cm)),
        ca.horzcat(1/4, 1/(4*l), 1/(4*l), -1/(4*Cm)),
        ca.horzcat(1/4, 1/(4*l), -1/(4*l), 1/(4*Cm)),
        ca.horzcat(1/4, -1/(4*l), 1/(4*l), 1/(4*Cm)))
    M = A@ca.vertcat(T, Mx, My, Mz)
    for i in range(4):
        M[i] = ca.if_else(M[i] < 0, 0, M[i])
    omega = ca.sqrt(M/Ct)
    f_alloc = ca.Function("control_allocation",
            [l, Cm, Ct, T, Mx, My, Mz],
            [omega],
            ['l', 'Cm', 'Ct', 'T', 'Mx', 'My', 'Mz'],
            ['omega']
            )
    return {
        "control_allocation": f_alloc
    }

def saturate(x, x_min, x_max):
    """
    saturate a vector
    """
    y = x
    for i in range(x.shape[0]):
        y[i] =  ca.if_else(x[i] > x_max[i], x_max[i], ca.if_else(x[i] < x_min[i], x_min[i], x[i]))
    return y

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
    i_max = ca.SX.sym('i_max', 3)

    # VARIABLES
    # -------------------------------
    omega = ca.SX.sym('omega', 3)
    omega_r = ca.SX.sym('omega_r', 3)
    i0 = ca.SX.sym('i0', 3)
    dt = ca.SX.sym('dt')

    # CALC
    # -------------------------------

    # actual attitude, expressed as quaternion
    e = omega_r - omega

    # integral action helps balance distrubance moments (e.g. center of gravity offset)
    i1 = saturate(i0 + e * dt, -i_max, i_max)

    M = kp * e + ki * i1

    # FUNCTION
    # -------------------------------
    f_attitude_rate_control = ca.Function(
        "attitude_rate_control",
        [kp, ki, i_max, omega, omega_r, i0, dt],
        [M, i1],
        ["kp", "ki", "i_max", "omega", "omega_r", "i0", "dt"],
        ["M", "i1"])

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
    z_i_2 = saturate(z_i_2,
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

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="rdd2.c", dest_dir=args.dest_dir)
    print("complete")
