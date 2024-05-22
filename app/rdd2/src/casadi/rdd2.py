#!/usr/bin/env python3
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import SO3Quat, SO3EulerB321

# parameters
thrust_delta = 0.1 # thrust delta from trim
thrust_trim = 0.5 # thrust trim
deg2rad = np.pi/180 # degree to radian
g = 9.8 # grav accel m/s^2
m = 2.0 # mass of vehicle
roll_rate_max = 60 # deg/s
Kp = 2 # position proportional gain
Kv = 2 # velocity proportional gain


def derive_joy_acro():
    w = ca.SX.sym('omega', 3)

    joy_roll = ca.SX.sym('joy_roll')
    joy_pitch = ca.SX.sym('joy_pitch')
    joy_yaw = ca.SX.sym('joy_yaw')
    joy_thrust = ca.SX.sym('joy_thrust')

    w[0] = -roll_rate_max*deg2rad*joy_roll
    w[1] = roll_rate_max*deg2rad*joy_pitch
    w[2] = roll_rate_max*deg2rad*joy_yaw

    thrust = joy_thrust * thrust_delta + thrust_trim

    f_joy_acro = ca.Function(
       "joy_acro",
       [joy_roll, joy_pitch, joy_yaw, joy_thrust],
       [w, thrust],
       ["joy_roll", "joy_pitch", "joy_yaw", "joy_thrust"],
       ["omega", "thrust"])

    return {
        "joy_acro": f_joy_acro
    }


def derive_quat_to_eulerB321():
    q_wb = ca.SX.sym('q', 4)
    X = SO3Quat.elem(q_wb)
    e = SO3EulerB321.from_Quat(X)

    f_quat_to_eulerB321 = ca.Function(
       "quat_to_eulerB321",
       [q_wb], [e.param[0], e.param[1], e.param[2]],
       ["q_wb"], ["yaw", "pitch", "roll"])

    return {
        "quat_to_eulerB321": f_quat_to_eulerB321
    }

def derive_eulerB321_to_quat():
    e = SO3EulerB321.elem(ca.SX.sym('e', 3))
    X = SO3Quat.from_Euler(e)
    f_eulerB321_to_quat = ca.Function(
       "eulerB321_to_quat",
       [e.param[0], e.param[1], e.param[2]], [X.param],
       ["yaw", "pitch", "roll"], ["q"])

    return {
        "eulerB321_to_quat": f_eulerB321_to_quat
    }


def derive_attitude_control():
    kp = ca.vertcat(4, 4, 2)

    # actual attitude, expressed as quaternion
    q = ca.SX.sym('q', 4)
    X = SO3Quat.elem(q)

    q_r = ca.SX.sym('q_r', 4)
    X_r = SO3Quat.elem(q_r)

    # Lie algebra
    e = (X.inverse() * X_r).log()  # angular velocity to get to desired att in 1 sec

    omega = kp * e.param # elementwise

    f_attitude_control = ca.Function(
        "attitude_control",
        [q, q_r],
        [omega],
        ["q", "q_r"],
        ["omega"])

    return {
        "attitude_control": f_attitude_control
    }


def derive_position_control():
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

    # normalized thrust vector, normalized by twice weight
    p_norm_max = 0.3
    p_term = -Kp * e_p / (2*g) - Kv * e_v / (2*g) + at_w / (2*g)
    p_norm = ca.norm_2(p_term)
    p_term = ca.if_else(p_norm > p_norm_max, p_norm_max*p_term/p_norm, p_term)

    # trim throttle
    T0 = zW / 2

    T = p_term + T0

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

    f_get_u = ca.Function(
        "position_control",
        [pt_w, vt_w, at_w, qc_wb.param, p_w, v_b, q_wb.param], [nT, qr_wb.param], 
        ['pt_w', 'vt_w', 'at_w', 'qc_wb', 'p_w', 'v_b', 'q_wb'], 
        ['nT', 'qr_wb'])
    
    return {
        "position_control" : f_get_u
    }


def generate_code(eqs: dict, filename, dest_dir: str, **kwargs):
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
    print("generating casadi equations")
    eqs = {}
    eqs.update(derive_attitude_control())
    eqs.update(derive_position_control())
    eqs.update(derive_eulerB321_to_quat())
    eqs.update(derive_quat_to_eulerB321())
    eqs.update(derive_joy_acro())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="rdd2.c", dest_dir="gen")
    print("complete")
