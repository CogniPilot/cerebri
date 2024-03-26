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


def derive_attitude_error():
    import casadi as ca

    # actual attitude, expressed as quaternion
    q = ca.SX.sym('q', 4)
    X = SO3Quat.elem(q)

    # in auto level mode
    # user will input desired roll angle and desired pitch angel with 
    # sticks, these desired angles will not approach roll/pitch = 90 deg
    # as the desired angles will be saturated
    yaw_r = ca.SX.sym('yaw_r')
    pitch_r = ca.SX.sym('pitch_r')
    roll_r = ca.SX.sym('roll_r')
    R = SO3Quat.from_Euler(SO3EulerB321.elem(ca.vertcat(yaw_r, pitch_r, roll_r)))

    # the difference between vehicle attitudes will use Lie log map
    # to get the angular velocity to get between the current and 
    # desired attitude in 1 sec, this will be used to drive the desired body 
    # rates for the inner rate loop

    # Lie algebra
    omega = (X.inverse() * R).log()  # angular velocity to get to desired att in 1 sec
    # input to acro (rate loop)

    f_attitude_error = ca.Function(
        "attitude_error",
        [q, yaw_r, pitch_r, roll_r],
        [omega.param],
        ["q", "yaw_r", "pitch_r", "roll_r"],
        ["omega"])


    e = SO3EulerB321.from_Quat(X).param

    f_quaternion_to_euler = ca.Function(
       "quaternion_to_euler",
       [q], [e], ["q"], ["e"])

    eqs = {
            "attitude_error": f_attitude_error,
            "quaternion_to_euler": f_quaternion_to_euler
            }
    return eqs

def velocity_control():
    #inputs: velocity trajectory, desired Yaw vel, dt, Kp, Kv
    #state inputs: position, orientation, velocity, and angular velocity
    #outputs: thrust force, angular errors
    vt = ca.SX.sym('vt', 3)
    yt = ca.SX.sym('yt')
    Kp = ca.SX.sym('Kp')
    Kv = ca.SX.sym('Kv')
    
    vel = ca.SX.sym('vel', 3)
    q = ca.SX.sym('q', 4)
    
    e_p = -vt
    e_v = vel - vt
    at = ca.SX(0)
    
    zW = ca.SX([0, 0, 1])
    
    Fd = - Kp * e_p - Kv * e_v + 2.0*9.8*zW + 2.0*at
    
    zB = at + ca.SX([0, 0, 9.8])
    zB = zB / ca.sqrt(ca.sumsqr(zB))
    
    u1 = ca.norm_2(Fd * zB)
    
    Zbd = Fd / ca.sqrt(ca.sumsqr(Fd))
    
    Xcd = ca.vertcat(ca.cos(yt), ca.sin(yt), ca.SX(0))
    
    Ybd = ca.cross(Zbd, Xcd)
    Ybd = Ybd / ca.norm_2(Ybd)
    
    Xbd = ca.cross(Ybd, Zbd)
    
    
    wrbd = ca.horzcat(Xbd, Ybd, Zbd)
    
    wRb = ca.vertcat(
        ca.horzcat(2*(q[0]**2+q[1]**2)-1, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2])),
        ca.horzcat(2*(q[1]*q[2]+q[0]*q[3]), 2*(q[0]**2+q[2]**2)-1, 2*(q[2]*q[3]-q[0]*q[1])),
        ca.horzcat(2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]-q[0]*q[1]), 2*(q[0]**2+q[3]**2)-1)
    )
    
    e_r = 0.5 * (ca.transpose(wrbd) @ wRb - ca.transpose(wRb) @ wrbd)
    e_r = ca.vertcat(e_r[2, 1], e_r[0, 2], e_r[1, 0])
    
    f_get_u = ca.Function(
        "velocity_control",
        [vt, yt, Kp, Kv, vel, q], [u1, e_r], 
        ['vt', 'yt', 'Kp', 'Kv', 'vel', 'q'], 
        ["u1", "e_r"])
    
    eqs = {
        "velocity_control" : f_get_u
    }

    return eqs

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
    eqs.update(derive_attitude_error())
    eqs.update(velocity_control())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="rdd2.c", dest_dir="gen")
    print("complete")
