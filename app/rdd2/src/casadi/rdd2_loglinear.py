import argparse
import os
import numpy as np
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import so3, SO3Quat, SO3EulerB321, SO3Dcm
from cyecca.models.bezier import derive_dcm_to_quat
from cyecca.lie.group_se23 import SE23Quat, se23
from cyecca.symbolic import SERIES
from cyecca.models.rdd2 import saturatem

# print("python: ", sys.executable)

# parameters
g = 9.8  # grav accel m/s^2
m = 2.0  # mass of vehicle
# thrust_delta = 0.9*m*g # thrust delta from trim
# thrust_trim = m*g # thrust trim
deg2rad = np.pi / 180  # degree to radian


def saturate(x, x_min, x_max):
    """
    saturate a vector
    """
    y = x
    for i in range(x.shape[0]):
        y[i] = ca.if_else(
            x[i] > x_max[i], x_max[i], ca.if_else(x[i] < x_min[i], x_min[i], x[i])
        )
    return y


def derive_se23_error():
    """
    SE2(3) Error
    """
    # actual pos and vel
    p_w = ca.SX.sym("p_w", 3)
    v_w = ca.SX.sym("v_w", 3)
    # actual attitude, expressed as quaternion
    q_wb = ca.SX.sym("q_wb", 4)
    X = SE23Quat.elem(ca.vertcat(p_w, v_w, q_wb))

    # reference input
    p_rw = ca.SX.sym("p_rw", 3)
    v_rw = ca.SX.sym("v_rw", 3)
    q_r = ca.SX.sym("q_r", 4)
    X_r = SE23Quat.elem(ca.vertcat(p_rw, v_rw, q_r))

    #  Left invariant error in Lie group and Lie algebra
    eta = X.inverse() * X_r
    zeta = eta.log()

    f_se23_error = ca.Function(
        "se23_error",
        [p_w, v_w, q_wb, p_rw, v_rw, q_r],
        [zeta.param],
        ["p_w", "v_w", "q_wb", "p_rw", "v_rw", "q_r"],
        ["zeta"],
    )

    eqs = {"se23_error": f_se23_error}
    return eqs


def derive_so3_attitude_control():
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

    omega = so3.elem(e.param).left_jacobian() @ ca.diag(kp) @ e.param  # elementwise

    # FUNCTION
    # -------------------------------
    f_attitude_control = ca.Function(
        "so3_attitude_control", [kp, q, q_r], [omega], ["kp", "q", "q_r"], ["omega"]
    )

    return {"so3_attitude_control": f_attitude_control}


def adC_matrix():
    adC = ca.SX(9, 9)
    adC[0, 3] = 1
    adC[1, 4] = 1
    adC[2, 5] = 1

    return adC


import numpy as np
from scipy.linalg import solve_continuous_are


def lqr(A, B, Q, R):
    """
    Solve the continuous time LQR controller for a system dx/dt = A x + B u.

    Returns K, X, eigVals
    - K: state feedback gain
    - X: solution to Riccati equation
    - eigVals: closed loop eigenvalues
    """
    # Solve the continuous-time Algebraic Riccati Equation (ARE)
    X = solve_continuous_are(A, B, Q, R)

    # Compute the LQR gain
    K = np.linalg.inv(R) @ B.T @ X

    # Closed loop eigenvalues
    eigVals = np.linalg.eigvals(A - B @ K)

    return K, X, eigVals


def se23_solve_control():
    A = -ca.DM(se23.elem(ca.vertcat(0, 0, 0, 0, 0, 9.8, 0, 0, 0)).ad() + adC_matrix())
    B = ca.DM.eye(9)
    # B = np.array(
    #     [
    #         [0, 0, 0, 0, 0, 0],  # vx
    #         [0, 0, 0, 0, 0, 0],  # vy
    #         [0, 0, 0, 0, 0, 0],  # vz
    #         [1, 0, 0, 0, 0, 0],  # ax
    #         [0, 1, 0, 0, 0, 0],  # ay
    #         [0, 0, 1, 0, 0, 0],  # az
    #         [0, 0, 0, 1, 0, 0],  # omega1
    #         [0, 0, 0, 0, 1, 0],  # omega2
    #         [0, 0, 0, 0, 0, 1],
    #     ]
    # )  # omega3 # control omega1,2,3, and az
    # Q = 100*ca.diag(ca.vertcat(10, 10, 10, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5))  # penalize state
    Q = 20 * np.eye(9)  # penalize state
    R = 1 * ca.DM.eye(9)  # penalize input
    K, _, _ = lqr(A, B, Q, R)
    K = -K
    BK = B @ K
    return A, K, BK, A + B @ K


def derive_outerloop_control():
    """
    Given the position, velocity ,and acceleration set points, find the
    desired attitude and thrust.
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym("thrust_trim")
    BK = ca.SX.sym("BK", 9, 9)
    ki = ca.SX.sym("ki", 3)
    integral_max = ca.SX.sym("integral_max", 3)

    # INPUT VARIABLES
    # -------------------------------

    # inputs: position trajectory, velocity trajectory, desired Yaw vel, dt
    # state inputs: position, orientation, velocity, and angular velocity
    # outputs: thrust force, angular errors
    zeta = ca.SX.sym("zeta", 9)
    at_w = ca.SX.sym("at_w", 3)
    q_wb = SO3Quat.elem(ca.SX.sym("q_wb", 4))  # orientation
    z_i = ca.SX.sym("z_i", 3)  # z velocity error integral
    dt = ca.SX.sym("dt")  # time step

    # CALC
    # -------------------------------
    # get control input
    u_zeta = -se23.elem(zeta).left_jacobian() @ BK @ zeta

    # attitude control
    u_omega = u_zeta[6:]

    # position control
    uv = u_zeta[0:3]
    ua = u_zeta[3:6]

    xW = ca.SX([1, 0, 0])
    yW = ca.SX([0, 1, 0])
    zW = ca.SX([0, 0, 1])

    # normalized thrust vector
    p_norm_max = 0.3 * m * g
    uv_w = q_wb @ uv
    ua_w = q_wb @ ua
    p_term = uv_w + ua_w + m * at_w
    p_norm = ca.norm_2(p_term)
    p_term = ca.if_else(p_norm > p_norm_max, p_norm_max * p_term / p_norm, p_term)

    # throttle integral
    z_i_2 = z_i + zeta[0:3] * dt
    z_i_2 = saturatem(
        z_i_2,
        -thrust_trim
        * ca.vertcat(
            integral_max[0] / ki[0], integral_max[1] / ki[1], integral_max[2] / ki[2]
        ),
        thrust_trim
        * ca.vertcat(
            integral_max[0] / ki[0], integral_max[1] / ki[1], integral_max[2] / ki[2]
        ),
    )

    # trim throttle
    T = p_term + thrust_trim * zW + ca.diag(ca.vertcat(ki[0], ki[1], ki[2])) @ z_i

    # thrust
    nT = ca.norm_2(T)

    # body up is aligned with thrust
    zB = ca.if_else(nT > 1e-3, T / nT, zW)

    # point y using desired camera direction
    ec = SO3EulerB321.from_Quat(q_wb)
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
    q_sp = SO3Quat.from_Matrix(Rd_wb)

    # FUNCTION
    # -------------------------------
    f_get_u = ca.Function(
        "se23_control",
        [thrust_trim, BK, ki, integral_max, zeta, at_w, q_wb.param, z_i, dt],
        [nT, z_i_2, q_sp.param],
        [
            "thrust_trim",
            "BK",
            "ki",
            "integral_max",
            "zeta",
            "at_w",
            "q_wb",
            "z_i",
            "dt",
        ],
        ["nT", "z_i_2", "q_sp"],
    )

    f_se23_attitude_control = ca.Function(
        "se23_attitude_control", [BK, zeta], [u_omega], ["BK", "zeta"], ["omega"]
    )

    return {
        "se23_control": f_get_u,
        "se23_attitude_control": f_se23_attitude_control,
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
    parser = argparse.ArgumentParser()
    parser.add_argument("dest_dir")
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))
    eqs = {}
    eqs.update(derive_so3_attitude_control())
    eqs.update(derive_outerloop_control())
    eqs.update(derive_se23_error())

    for name, eq in eqs.items():
        print("eq: ", name)

    generate_code(eqs, filename="rdd2_loglinear.c", dest_dir=args.dest_dir)
    print("complete")
