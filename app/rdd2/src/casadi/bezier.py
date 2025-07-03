import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import SO3Quat, SO3EulerB321, SO3Dcm

print("python: ", sys.executable)

g = 9.8  # grav accel m/s^2
m = 2.0  # mass of vehicle
J_xx = 0.0216666
J_yy = 0.0216666
J_zz = 0.04
J_xz = 0


class Bezier:
    """
    https://en.wikipedia.org/wiki/B%C3%A9zier_curve
    """

    def __init__(self, P: ca.SX, T: float):
        self.P = P
        self.m = P.shape[0]
        self.n = P.shape[1] - 1
        self.T = T

    def eval(self, t):
        # https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        beta = t / self.T
        A = ca.SX(self.P)
        for j in range(1, self.n + 1):
            for k in range(self.n + 1 - j):
                A[:, k] = A[:, k] * (1 - beta) + A[:, k + 1] * beta
        return A[:, 0]

    def deriv(self, m=1):
        D = ca.SX(self.P)
        for j in range(0, m):
            D = (self.n - j) * ca.horzcat(
                *[D[:, i + 1] - D[:, i] for i in range(self.n - j)]
            )
        return Bezier(D / self.T**m, self.T)


def derive_bezier7():
    n = 8
    T = ca.SX.sym("T")
    t = ca.SX.sym("t")
    P = ca.SX.sym("P", 1, n)
    B = Bezier(P, T)

    # derivatives
    B_d = B.deriv()
    B_d2 = B_d.deriv()
    B_d3 = B_d2.deriv()
    B_d4 = B_d3.deriv()

    # boundary conditions

    # trajectory
    p = B.eval(t)
    v = B_d.eval(t)
    a = B_d2.eval(t)
    j = B_d3.eval(t)
    s = B_d4.eval(t)
    r = ca.vertcat(p, v, a, j, s)

    # given position/velocity boundary conditions, solve for bezier points
    wp_0 = ca.SX.sym("p0", 4, 1)  # pos/vel/acc/jerk at waypoint 0
    wp_1 = ca.SX.sym("p1", 4, 1)  # pos/vel/acc/jerk at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B_d2.eval(0), wp_0[2])]  # accel @ wp0
    constraints += [(B_d3.eval(0), wp_0[3])]  # jerk @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1
    constraints += [(B_d2.eval(0), wp_1[2])]  # accel @ wp1
    constraints += [(B_d3.eval(0), wp_1[3])]  # jerk @ wp1

    assert len(constraints) == n

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv @ b).T

    functions = [
        ca.Function(
            "bezier7_solve", [wp_0, wp_1, T], [P_sol], ["wp_0", "wp_1", "T"], ["P"]
        ),
        ca.Function("bezier7_traj", [t, T, P], [r], ["t", "T", "P"], ["r"]),
    ]

    return {f.name(): f for f in functions}


def derive_bezier3():
    n = 4
    T = ca.SX.sym("T")
    t = ca.SX.sym("t")
    P = ca.SX.sym("P", 1, n)
    B = Bezier(P, T)

    # derivatives
    B_d = B.deriv()
    B_d2 = B_d.deriv()
    B_d3 = B_d2.deriv()
    B_d4 = B_d3.deriv()

    # boundary conditions

    # trajectory
    p = B.eval(t)
    v = B_d.eval(t)
    a = B_d2.eval(t)
    r = ca.vertcat(p, v, a)

    # given position/velocity boundary conditions, solve for bezier points
    wp_0 = ca.SX.sym("p0", 2, 1)  # pos/vel at waypoint 0
    wp_1 = ca.SX.sym("p1", 2, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1

    assert len(constraints) == n

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv @ b).T

    functions = [
        ca.Function(
            "bezier3_solve",
            [
                wp_0,
                wp_1,
                T,
            ],
            [P_sol],
            [
                "wp_0",
                "wp_1",
                "T",
            ],
            ["P"],
        ),
        ca.Function(
            "bezier3_traj",
            [
                t,
                T,
                P,
            ],
            [r],
            [
                "t",
                "T",
                "P",
            ],
            ["r"],
        ),
    ]

    return {f.name(): f for f in functions}


def derive_dcm_to_quat():
    R = SO3Dcm.elem(ca.SX.sym("R", 9))
    q = SO3Quat.from_Dcm(R)

    functions = [ca.Function(
        "dcm_to_quat",
        [
            R.param,
        ],
        [q.param],
        [
            "R",
        ],
        ["q"],
        )
    ]

    return {f.name(): f for f in functions}


def derive_ref():
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Symbols and Parameters

    tol = 1e-6  # tolerance for singularities

    # flat output (input variables from trajectory planner)
    p_e = ca.SX.sym("p_e", 3)  # position
    v_e = ca.SX.sym("v_e", 3)  # velocity
    a_e = ca.SX.sym("a_e", 3)  # accel
    j_e = ca.SX.sym("j_e", 3)  # jerk
    s_e = ca.SX.sym("s_e", 3)  # snap

    psi = ca.SX.sym("psi")  # desired heading direction
    psi_dot = ca.SX.sym("psi_dot")  # derivative of desired heading
    psi_ddot = ca.SX.sym("psi_ddot")  # second derivative of desired heading

    # unit vectors xh = xb_b = xe_e etc.
    xh = ca.SX([1, 0, 0])
    yh = ca.SX([0, 1, 0])
    zh = ca.SX([0, 0, 1])

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Solve for C_be

    # acceleration
    thrust_e = m * (g * zh - a_e)

    T = ca.norm_2(thrust_e)
    T = ca.if_else(T > tol, T, tol)  # can have singularity when T = 0, this prevents it

    zb_e = thrust_e / T

    # desired heading direction
    xc_e = ca.cos(psi) * xh + ca.sin(psi) * yh

    yb_e = ca.cross(zb_e, xc_e)
    N_yb_e = ca.norm_2(yb_e)
    yb_e = ca.if_else(
        N_yb_e > tol, yb_e / N_yb_e, yh
    )  # normalize y_b, can have singularity when z_b and x_c aligned
    xb_e = ca.cross(yb_e, zb_e)

    # T_dot = ca.dot(m*s_e, zb_e)
    C_be = ca.hcat([xb_e, yb_e, zb_e])

    dcm_quat = derive_dcm_to_quat()
    quat = dcm_quat["dcm_to_quat"](SO3Dcm.from_Matrix(C_be).param)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Solve for omega_eb_b

    # note h_omega z_b component can be ignored with dot product below
    t2_e = m / T * j_e
    p = ca.dot(t2_e, yb_e)
    q = -ca.dot(t2_e, xb_e)

    C_eb = C_be.T

    # solve for euler angles based on DCM
    theta = ca.asin(-C_eb[2, 0])  # check if transpose
    phi = ca.if_else(
        ca.fabs(ca.fabs(theta) - ca.pi / 2) < tol, 0, ca.atan2(C_eb[2, 1], C_eb[2, 2])
    )

    # solve for r
    cos_phi = ca.cos(phi)
    cos_phi = ca.if_else(ca.fabs(cos_phi) > tol, cos_phi, 0)
    r = (
        -q * ca.tan(phi) + ca.cos(theta) * psi_dot / cos_phi
    )  # from R_solve below, singularity at phi=pi

    T_dot = -ca.dot(m * j_e, zb_e)

    # Mellinger approach
    # yc_e = ca.cross(xc_e, zh)
    # R_sol = ca.inv(ca.horzcat(xb_e, yc_e, zh))@C_be
    # R_sol[2, 0]*p + R_sol[2, 1]*q + R_sol[2, 0]*r = psi_dot
    # r2 = (psi_dot - R_sol[2, 0]*p + R_sol[2, 1]*q)/R_sol[2, 0]

    omega_eb_b = p * xh + q * yh + r * zh

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Solve for omega_dot_eb_b

    omega_eb_b_cross_zh = ca.cross(omega_eb_b, zh)

    coriolis_b = 2 * T_dot / T * omega_eb_b_cross_zh
    centrip_b = ca.cross(omega_eb_b, omega_eb_b_cross_zh)

    q_dot = -m / T * ca.dot(s_e, xb_e) - ca.dot(coriolis_b, xh) - ca.dot(centrip_b, xh)
    p_dot = m / T * ca.dot(s_e, yb_e) + ca.dot(coriolis_b, yh) + ca.dot(centrip_b, yh)

    omega_eb_e = C_be @ omega_eb_b
    omega_ec_e = psi_dot * zh

    theta_dot = (q - ca.sin(phi) * ca.cos(theta) * psi_dot) / ca.cos(phi)
    phi_dot = p + ca.sin(theta) * psi_dot

    zc_e = zh  # c frame rotates about ze so zc_c = zc_e = zh
    yc_e = ca.cross(zc_e, xc_e)
    T1 = ca.inv(ca.horzcat(xb_e, yc_e, zh))
    A = T1 @ C_be
    b = -T1 @ (
        ca.cross(omega_eb_e, phi_dot * xb_e) + ca.cross(omega_ec_e, theta_dot * yc_e)
    )
    r_dot = (psi_ddot - A[2, 0] * p_dot - A[2, 1] * q_dot - b[2]) / A[2, 2]

    omega_dot_eb_b = p_dot * xh + q_dot * yh + r_dot * zh

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Solve for Inputs

    J = ca.SX(3, 3)
    J[0, 0] = J_xx
    J[1, 1] = J_yy
    J[2, 2] = J_zz
    J[0, 2] = J[2, 0] = J_xz

    M_b = J @ omega_dot_eb_b + ca.cross(omega_eb_b, J @ omega_eb_b)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Code Generation

    v_b = C_eb @ v_e
    functions = [ca.Function(
        "f_ref",
        [
            psi,
            psi_dot,
            psi_ddot,
            v_e,
            a_e,
            j_e,
            s_e,
        ],
        [quat, omega_eb_b, M_b],
        [
            "psi",
            "psi_dot",
            "psi_ddot",
            "v_e",
            "a_e",
            "j_e",
            "s_e",
        ],
            ["quat", "omega_eb_b", "M_b"],
        ),
    ]
    return {f.name(): f for f in functions}


def derive_multirotor():
    n1 = 8
    bezier_7 = derive_bezier7()
    bezier_7_solve = bezier_7["bezier7_solve"]
    bezier_7_traj = bezier_7["bezier7_traj"]

    n2 = 4
    bezier_3 = derive_bezier3()
    bezier_3_solve = bezier_3["bezier3_solve"]
    bezier_3_traj = bezier_3["bezier3_traj"]

    T = ca.SX.sym("T")
    t = ca.SX.sym("t")

    PX = ca.SX.sym("PX", 1, n1)
    PY = ca.SX.sym("PY", 1, n1)
    PZ = ca.SX.sym("PZ", 1, n1)
    traj_x = bezier_7_traj(t, T, PX)
    traj_y = bezier_7_traj(t, T, PY)
    traj_z = bezier_7_traj(t, T, PZ)

    Ppsi = ca.SX.sym("Ppsi", 1, n2)
    traj_psi = bezier_3_traj(t, T, Ppsi)

    x = traj_x[0]
    vx = traj_x[1]
    ax = traj_x[2]
    jx = traj_x[3]
    sx = traj_x[4]

    y = traj_y[0]
    vy = traj_y[1]
    ay = traj_y[2]
    jy = traj_y[3]
    sy = traj_y[4]

    z = traj_z[0]
    vz = traj_z[1]
    az = traj_z[2]
    jz = traj_z[3]
    sz = traj_z[4]

    # camera orientation what vehicle wants to look at -> orientation sp
    psi = traj_psi[0]
    dpsi = traj_psi[1]  # yaw rate
    ddpsi = traj_psi[2]

    v = ca.vertcat(vx, vy, vz)
    a = ca.vertcat(ax, ay, az)
    j = ca.vertcat(jx, jy, jz)
    s = ca.vertcat(sx, sy, sz)

    functions = [ca.Function(
        "bezier_multirotor",
        [
            t,
            T,
            PX,
            PY,
            PZ,
            Ppsi,
        ],
        [x, y, z, psi, dpsi, ddpsi, v, a, j, s],
        [
            "t",
            "T",
            "PX",
            "PY",
            "PZ",
            "Ppsi",
        ],
        ["x", "y", "z", "psi", "psidot", "psiddot", "v", "a", "j", "s"],
    )
    ]

    return {f.name(): f for f in functions}


def derive_eulerB321_to_quat():
    """
    eulerB321 to quaternion converion
    """

    # INPUTS
    # -------------------------------
    e = SO3EulerB321.elem(ca.SX.sym("e", 3))

    # CALC
    # -------------------------------
    X = SO3Quat.from_Euler(e)

    # FUNCTION
    # -------------------------------
    f_eulerB321_to_quat = ca.Function(
        "eulerB321_to_quat",
        [
            e.param[0],
            e.param[1],
            e.param[2],
        ],
        [X.param],
        [
            "yaw",
            "pitch",
            "roll",
        ],
        ["q"],
    )

    return {"eulerB321_to_quat": f_eulerB321_to_quat}


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
    parser = argparse.ArgumentParser()
    parser.add_argument("dest_dir")
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))
    eqs = {}
    eqs.update(derive_bezier7())
    eqs.update(derive_bezier3())
    eqs.update(derive_dcm_to_quat())
    eqs.update(derive_ref())
    eqs.update(derive_multirotor())

    for name, eq in eqs.items():
        print("eq: ", name)

    generate_code(eqs, filename="bezier.c", dest_dir=args.dest_dir)
    print("complete")
