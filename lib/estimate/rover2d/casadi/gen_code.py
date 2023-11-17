#!/usr/bin/env python3

from cyecca import lie
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import pathlib

# define symbols
x = ca.SX.sym("x")  # x position in world frame of rear axle (north)
y = ca.SX.sym("y")  # y position in world frame of rear axle (east)
theta = ca.SX.sym("theta")  # angular heading in world frame (rotation about down)
u = ca.SX.sym("u")  # forward velocity, along body x
omega = ca.SX.sym("omega")  # angular velocity around z axis
dt = ca.SX.sym("dt")  # time stemp

G = lie.SE2
X = G.elem(ca.vertcat(x, y, theta))
v = G.algebra.elem(ca.vertcat(u, 0, omega))

X1 = X + v
f_predict = ca.Function(
    "predict", [X.param, omega, u], [X1.param], ["x0", "omega", "u"], ["x1"]
)

eqs = {"rover2d": {"predict": f_predict}}

def generate_code(eqs: dict, dest_dir: str, **kwargs):
    dest_dir = pathlib.Path(dest_dir)
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

    for name, eqs in eqs.items():
        filename = "{:s}.c".format(name)
        gen = ca.CodeGenerator(filename, p)
        for f_name in eqs:
            gen.add(eqs[f_name])

        dest_dir = pathlib.Path(dest_dir)
        dest_dir.mkdir(exist_ok=True)

        gen.generate(str(dest_dir) + pathlib.os.sep)


generate_code(eqs, ".")
