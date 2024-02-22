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

    eqs = {"attitude_error": f_attitude_error}
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

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="rdd2.c", dest_dir="gen")
    print("complete")
