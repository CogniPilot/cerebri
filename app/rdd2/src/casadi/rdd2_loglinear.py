import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie
from cyecca.lie.group_so3 import so3, SO3Quat, SO3EulerB321
from cyecca.lie.group_se23 import (
    SE23Quat,
    se23,
    SE23LieGroupElement,
    SE23LieAlgebraElement,
)
from cyecca.symbolic import SERIES

print("python: ", sys.executable)

# parameters
g = 9.8  # grav accel m/s^2
m = 2.24  # mass of vehicle
# thrust_delta = 0.9*m*g # thrust delta from trim
# thrust_trim = m*g # thrust trim
deg2rad = np.pi / 180  # degree to radian

# attitude rate loop
rollpitch_rate_max = 30  # deg/s
yaw_rate_max = 60  # deg/s

# done kp_rollpitch_rate = 0.3
# done ki_rollpitch_rate = 0.05
# done rollpitch_rate_integral_max = 1.0

# done kp_yaw_rate = 0.3
# done ki_yaw_rate = 0.05
# done yaw_rate_integral_max = 1.0

# attitude loop
rollpitch_max = 20  # deg
# kp_rollpitch = 2
# kp_yaw = 1

# position loop
kp_pos = 0.5  # position proportional gain
kp_vel = 2.0  # velocity proportional gain
# pos_sp_dist_max = 2 # position setpoint max distance
# vel_max = 2.0 # max velocity command
z_integral_max = 0  # 5.0
ki_z = 0.05  # velocity z integral gain


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
        "so3_attitude_control",
        [
            kp,
            q,
            q_r,
        ],
        [omega],
        [
            "kp",
            "q",
            "q_r",
        ],
        ["omega"],
    )

    return {"so3_attitude_control": f_attitude_control}


def derive_outerloop_control():
    """
    Given the position, velocity ,and acceleration set points, find the
    desired attitude and thrust.
    """

    # INPUT CONSTANTS
    # -------------------------------
    thrust_trim = ca.SX.sym("thrust_trim")
    kp = ca.SX.sym("kp", 3)

    # INPUT VARIABLES
    # -------------------------------

    # inputs: position trajectory, velocity trajectory, desired Yaw vel, dt
    # state inputs: position, orientation, velocity, and angular velocity
    # outputs: thrust force, angular errors
    zeta = ca.SX.sym("zeta", 9)
    at_w = ca.SX.sym("at_w", 3)
    qc_wb = SO3Quat.elem(ca.SX.sym("qc_wb", 4))  # camera orientation
    z_i = ca.SX.sym("z_i")  # z velocity error integral
    dt = ca.SX.sym("dt")  # time step

    # CALC
    # -------------------------------
    # get control input
    K_se23 = ca.diag(ca.vertcat(kp_pos, kp_pos, kp_pos, kp_vel, kp_vel, kp_vel, kp))
    u_zeta = se23.elem(zeta).left_jacobian() @ K_se23 @ zeta

    # attitude control
    u_omega = u_zeta[6:]

    # position control
    u_v = u_zeta[0:3]
    u_a = u_zeta[3:6]

    xW = ca.SX([1, 0, 0])
    yW = ca.SX([0, 1, 0])
    zW = ca.SX([0, 0, 1])

    # F = - Kp ep - Kv ev + mg zW + m at_w
    # F = - m * Kp' ep - m * Kv' * ev + mg zW + m at_w
    # Force is normalized by the weight (mg)

    # normalized thrust vector
    p_norm_max = 0.3 * m * g
    p_term = u_v + u_a + m * at_w
    p_norm = ca.norm_2(p_term)
    p_term = ca.if_else(p_norm > p_norm_max, p_norm_max * p_term / p_norm, p_term)

    # throttle integral
    z_i_2 = z_i + zeta[2] * dt
    z_i_2 = saturate(z_i_2, -ca.vertcat(z_integral_max), ca.vertcat(z_integral_max))

    # trim throttle
    T = p_term + thrust_trim * zW + ki_z * z_i * zW

    # thrust
    nT = ca.norm_2(T)

    # body up is aligned with thrust
    zB = ca.if_else(nT > 1e-3, T / nT, zW)

    # point y using desired camera direction
    ec = SO3EulerB321.from_Quat(qc_wb)
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
    qr_wb = SO3Quat.from_Matrix(Rd_wb)

    # FUNCTION
    # -------------------------------
    f_get_u = ca.Function(
        "se23_position_control",
        [
            thrust_trim,
            kp,
            zeta,
            at_w,
            qc_wb.param,
            z_i,
            dt,
        ],
        [nT, qr_wb.param, z_i_2],
        [
            "thrust_trim",
            "kp",
            "zeta",
            "at_w",
            "qc_wb",
            "z_i",
            "dt",
        ],
        ["nT", "qr_wb", "z_i_2"],
    )

    f_se23_attitude_control = ca.Function(
        "se23_attitude_control",
        [
            kp,
            zeta,
        ],
        [u_omega],
        [
            "kp",
            "zeta",
        ],
        ["omega"],
    )

    return {
        "se23_position_control": f_get_u,
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
