import argparse 
import os
import sys
import math
import sympy
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import scipy.signal
import cyecca.lie as lie
from cyecca.symbolic import sympy_to_casadi
from cyecca.lie.group_so3 import SO3Quat, SO3EulerB321

print('python: ', sys.executable)

def continuous_to_discrete_controllable_realization(G, name):
    s = sympy.symbols('s')
    z = sympy.symbols('z')
    T = sympy.symbols('T')
    BLT = 2/T *(z-1)/(z+1)

    Gz = G.subs(s, BLT)
    Gz = Gz.simplify()

    num = sympy.poly(sympy.numer(Gz).expand().collect(z), z)
    den = sympy.poly(sympy.denom(Gz).expand().collect(z), z)
    q, r = sympy.div(num, den, z)

    n = len(den.coeffs()) - 1
    A = sympy.zeros(n, n)
    B = sympy.zeros(n, 1)
    C = sympy.zeros(1, n)
    D = sympy.zeros(1, 1)

    a = den.coeffs()[0]
    for i in range(n):
        for j in range(n):
            if i + 1 == j:
                A[i, j] = 1
            elif i == n - 1:
                A[i, j] = -sympy.simplify(den.coeffs()[n - j]/a)

    D[0, 0] = q/1
    B[n-1, 0] = 1

    for j in range(n):
        C[0, j] = sympy.simplify(r.coeffs()[j - 1]/a)

    ss = sympy.Matrix.vstack(sympy.Matrix.hstack(A, B), sympy.Matrix.hstack(C, D))

    ca_ss, ca_ss_syms = sympy_to_casadi(ss, cse=True)
    f_ss = ca.Function('f_ss', list(ca_ss_syms.values()), [ca_ss], list(ca_ss_syms.keys()), ['ss'])

    x = ca.SX.sym('x', 2)
    u = ca.SX.sym('u', 1)

    ss_eval = f_ss(*ca_ss_syms.values())
    A = ss_eval[:n, :n]
    B = ss_eval[:n, n:]
    C = ss_eval[n:, :n]
    D = ss_eval[n:, n:]


    x1 = A@x + B@u
    y = C@x + D@u

    f = ca.Function(name, list(ca_ss_syms.values()) + [x, u], [x1, y], list(ca_ss_syms.keys()) + ['x', 'u'], ['x_1', 'y'])

    return f


def derive_butterworth_2_filter():
    """
    Derive 2nd order butterworth filter.
    """

    s = sympy.symbols('s')
    T = sympy.symbols('T', real=True)
    wn = sympy.symbols('w_n', real=True)
    tau = sympy.symbols('tau', real=True)
    zeta = sympy.sqrt(2)/2
    G = wn**2/(s**2 + 2*zeta*wn*s + wn**2)
    f = continuous_to_discrete_controllable_realization(G, 'butterworth_2_filter')
    return {
        'butterworth_2_filter': f
    }


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
    import argparse 
    parser = argparse.ArgumentParser()
    parser.add_argument('dest_dir')
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))
    eqs = {}
    eqs.update(derive_butterworth_2_filter())
    eqs.update(derive_eulerB321_to_quat())
    eqs.update(derive_quat_to_eulerB321())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="common.c", dest_dir=args.dest_dir)
    print("complete")
