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

def derive_quat_to_eulerB321():
    """
    quaternion to eulerB321 converion
    """

    # INPUTS
    # -------------------------------
    q_wb = ca.SX.sym('q', 4)
    X = SO3Quat.elem(q_wb)
    e = SO3EulerB321.from_Quat(X)

    # FUNCTION
    # -------------------------------
    f_quat_to_eulerB321 = ca.Function(
       "quat_to_eulerB321",
       [q_wb], [e.param[0], e.param[1], e.param[2]],
       ["q_wb"], ["yaw", "pitch", "roll"])

    return {
        "quat_to_eulerB321": f_quat_to_eulerB321
    }

def derive_eulerB321_to_quat():
    """
    eulerB321 to quaternion converion
    """

    # INPUTS
    # -------------------------------
    e = SO3EulerB321.elem(ca.SX.sym('e', 3))

    # CALC
    # -------------------------------
    X = SO3Quat.from_Euler(e)

    # FUNCTION
    # -------------------------------
    f_eulerB321_to_quat = ca.Function(
       "eulerB321_to_quat",
       [e.param[0], e.param[1], e.param[2]], [X.param],
       ["yaw", "pitch", "roll"], ["q"])

    return {
        "eulerB321_to_quat": f_eulerB321_to_quat
    }


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
    print("generating casadi equations")
    eqs = {}
    eqs.update(derive_eulerB321_to_quat())
    eqs.update(derive_quat_to_eulerB321())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="common.c", dest_dir="gen")
    print("complete")
