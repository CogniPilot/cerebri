import argparse
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import casadi as ca
import cyecca.lie as lie

print('python: ', sys.executable)

class Bezier:
    """
    https://en.wikipedia.org/wiki/B%C3%A9zier_curve
    """

    def __init__(self, P: ca.SX, T: float):
        self.P = P
        self.m = P.shape[0]
        self.n = P.shape[1]-1
        self.T = T
    
    def eval(self, t):
        #https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
        beta = t/self.T
        A = ca.SX(self.P)
        for j in range(1, self.n + 1):
            for k in range(self.n + 1 - j):
                A[:, k] = A[:, k] * (1 - beta) + A[:, k + 1] * beta
        return A[:, 0]
    
    def deriv(self, m=1):
        D = ca.SX(self.P)
        for j in range(0, m):
            D = (self.n - j)*ca.horzcat(*[ D[:, i+1] - D[:, i]
                for i in range(self.n - j) ])
        return Bezier(D/self.T**m, self.T)

def derive_bezier6():
    n = 6
    T = ca.SX.sym('T')
    t = ca.SX.sym('t')
    P = ca.SX.sym('P', 1, n)
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
    wp_0 = ca.SX.sym('p0', 2, 1)  # pos/vel at waypoint 0
    wp_1 = ca.SX.sym('p1', 2, 1)  # pos/vel at waypoint 1

    constraints = []
    constraints += [(B.eval(0), wp_0[0])]  # pos @ wp0
    constraints += [(B_d.eval(0), wp_0[1])]  # vel @ wp0
    constraints += [(B_d2.eval(0), 0)]  # zero accel @ wp0
    constraints += [(B.eval(T), wp_1[0])]  # pos @ wp1
    constraints += [(B_d.eval(T), wp_1[1])]  # vel @ wp1
    constraints += [(B_d2.eval(T), 0)]  # zero accel @ wp1

    assert len(constraints) == 6

    Y = ca.vertcat(*[c[0] for c in constraints])
    b = ca.vertcat(*[c[1] for c in constraints])
    A = ca.jacobian(Y, P)
    A_inv = ca.inv(A)
    P_sol = (A_inv@b).T

    functions = [
        ca.Function('bezier6_solve', [wp_0, wp_1, T], [P_sol],
            ['wp_0', 'wp_1', 'T'], ['P']),
        ca.Function('bezier6_traj', [t, T, P], [r],
            ['t', 'T', 'P'], ['r']),
    ]

    return { f.name(): f for f in functions }


def derive_rover():
    n = 6
    bz = derive_bezier6()
    bezier_6_solve = bz['bezier6_solve']
    bezier_6_traj = bz['bezier6_traj']

    L = ca.SX.sym('L')  # wheel base
    w = ca.SX.sym('w')  # wheel separation
    T = ca.SX.sym('T')
    t = ca.SX.sym('t')

    PX = ca.SX.sym('PX', 1, n)
    PY = ca.SX.sym('PY', 1, n)
    traj_x = bezier_6_traj(t, T, PX)
    traj_y = bezier_6_traj(t, T, PY)

    x = traj_x[0]
    vx = traj_x[1]
    ax = traj_x[2]

    y = traj_y[0]
    vy = traj_y[1]
    ay = traj_y[2]

    psi = ca.arctan2(vy, vx)
    V = ca.sqrt(vx**2 + vy**2)
    omega = (vx*ay - vy*ax)/V**2

    # ackermann steering
    omega_in = ca.SX.sym('omega')
    V_in = ca.SX.sym('V')
    delta = ca.atan(L*omega_in/V_in)

    # differential drive
    h = ca.sqrt(L**2 + w**2)/2  # hypontenuse
    Vt = h*omega_in;  # tangent velocity
    theta = ca.atan(L/w)
    Vw = Vt/ca.cos(theta) # wheel velocity

    functions = [
        ca.Function(
            'bezier6_rover', [t, T, PX, PY], [x, y, psi, V, omega],
            ['t', 'T', 'PX', 'PY'], ['x', 'y', 'psi', 'V', 'omega']),
        ca.Function(
            'ackermann_steering', [L, omega_in, V_in], [delta],
            ['L', 'omega', 'V'], ['delta']),
        ca.Function(
            'differential_steering', [L, omega_in, w], [Vw],
            ['L', 'omega', 'w'], ['Vw'])
    ]

    return { f.name(): f for f in functions }


def test_bezier():
    """
    check bezier with numerical derivatives
    """
    bezier_6= derive_bezier6()
    T0 = 4
    t0 = np.linspace(0, T0, 1000)
    P0 = bezier_6['bezier6_solve']([0, 0], [1, 0], T0)
    traj = np.array(bezier_6['bezier6_traj'](np.array([t0]), T0, P0)).T
    p = traj[:, 0]
    v = traj[:, 1]
    a = traj[:, 2]

    v2 = np.diff(p.reshape(-1), axis=0)/np.diff(t0, axis=0)
    a2 = np.diff(v2.reshape(-1), axis=0)/np.diff(t0[1:], axis=0)
    
    assert np.max(v[1:] - v2) < 1e-1
    assert np.max(a[2:] - a2) < 1e-1

    plt.plot(t0, p, label='p')
    plt.plot(t0, v, label='v', linewidth=3, color='yellow')
    plt.plot(t0, a, label='a', color='orange', linewidth=3)
    plt.plot(t0[1:], v2, label='v2', color='red')
    plt.plot(t0[2:], a2, label='a2', color='green')
    plt.legend()
    plt.grid() 


def rover_plan():
    T0 = 2
    bezier_6 = derive_bezier6()

    bc = np.array([
        [ # position
            [1, 2],  # wp0, x, y
            [2, 3]   # wp1, x, y
        ],
        [ # velocity
            [1, 0],
            [0, 1]
        ]
    ])

    t0 = np.linspace(0, T0, 1000)
    PX = bezier_6['bezier6_solve'](bc[:, 0, 0], bc[:, 1, 0], T0)
    traj_x = np.array(bezier_6['bezier6_traj'](np.array([t0]), T0, PX)).T

    PY = bezier_6['bezier6_solve'](bc[:, 0, 1], bc[:, 1, 1], T0)
    traj_y = np.array(bezier_6['bezier6_traj'](np.array([t0]), T0, PY)).T

    x = traj_x[:, 0]
    vx = traj_x[:, 1]
    ax = traj_x[:, 2]

    y = traj_y[:, 0]
    vy = traj_y[:, 1]
    ay = traj_y[:, 2]

    psi = np.arctan2(vy, vx)
    V = np.sqrt(vx**2 + vy**2)
    omega = (vx*ay - vy*ax)/V

    if True:

        plt.plot(y, x)
        plt.axis('equal');
        plt.grid()

        plt.figure()
        plt.title('heading angle')
        plt.plot(t0, psi)
        plt.grid()

        plt.figure()
        plt.title('velocity')
        plt.plot(t0, V)
        plt.grid()

        plt.figure()
        plt.title('angular velocity')
        plt.plot(t0, omega)
        plt.grid()


def derive_se2():
    # derive se2_error
    p = lie.SE2.elem(ca.SX.sym('p', 3))
    r = lie.SE2.elem(ca.SX.sym('r', 3))
    se2_error = ca.Function('se2_error',
            [p.param, r.param],
            [lie.SE2.log(p.inverse()*r).param],
            ['p', 'r'], ['error'])

    # SE(2) differential correction term
    e = ca.SX.sym('e', 3)
    x = e[0]
    y = e[1]
    theta = e[2]   

    sin_theta = ca.sin(theta)
    cos_theta = ca.cos(theta)

    sin_2theta = 2*sin_theta*cos_theta
    cos_2theta = 2*cos_theta*cos_theta - 1

    a = ca.if_else(ca.fabs(theta) > 1e-3, theta*sin_theta/(2*(cos_theta - 1)), -1 + theta**2/12 + theta**4/720)
    b = theta/2
    c = ca.if_else(ca.fabs(theta) > 1e-3,(theta*x*sin_theta + (1 - cos_theta)*(theta*y - 2*x))/(2*theta*(1 - cos_theta)), y/2 - theta*x/12 + theta**3*x/720)
    d = ca.if_else(ca.fabs(theta) > 1e-3, (theta*y*sin_theta + (1 - cos_theta)*(-theta*x - 2*y))/(2*theta*(1 - cos_theta)), -x/2 - theta*y/12 - theta**3*y/720)
    
    U = ca.SX.zeros(3, 3)
    U[0, 0] = a
    U[0, 1] = -b
    U[0, 2] = c
    U[1, 0] = b
    U[1, 1] = a
    U[1, 2] = d
    U[2, 0] = 0
    U[2, 1] = 0
    U[2, 2] = -1

    # SE(2) differential correction inverse
    a = ca.if_else(ca.fabs(theta) > 1e-3, sin_theta/theta, 1 - theta**2/6 + theta**4/120)
    b = ca.if_else(ca.fabs(theta) > 1e-3, -(1  - cos_theta)/theta, theta/2 - theta**3/24)
    c = ca.if_else(ca.fabs(theta) > 1e-3, -(x*(theta*cos_theta - theta + sin_theta - sin_2theta/2) + y*(2*cos_theta - cos_2theta/2 - 3/2))/(theta**2*(1 - cos_theta)),
                y/2 + theta*x/6 - theta**2*y/24 - theta**3*x/120 + theta**4*y/720)
    d = ca.if_else(ca.fabs(theta) > 1e-3, -(x*(-2*cos_theta + cos_2theta/2 + 3/2) + y*(theta*cos_theta - theta + sin_theta - sin_2theta/2))/(theta**2*(1 - cos_theta)),
                -x/2 + theta*y/6 + theta**2*x/24 - theta**3*y/120 - theta**4*x/720)
    U_inv = ca.SX.zeros(3, 3)
    U_inv[0, 0] = a
    U_inv[0, 1] = b
    U_inv[0, 2] = c
    U_inv[1, 0] = -b
    U_inv[1, 1] = a
    U_inv[1, 2] = d
    U_inv[2, 2] = 1
    U_inv = -U_inv
    
    # SE(2) U inv series form
    ad_zeta = ca.SX.zeros(3, 3)
    ad_zeta[0, 1] = -theta
    ad_zeta[1, 0] = theta
    ad_zeta[0, 2] = y
    ad_zeta[1, 2] = -x
    n_series = 100

    U_inv_series = ca.SX.eye(3)
    ad_zeta_k = ca.SX.eye(3)
    for k in range(1, 10):
        ad_zeta_k = ad_zeta_k@ad_zeta
        U_inv_series += ad_zeta_k/math.factorial(k+1)
    U_inv_series = -U_inv_series
    
    f_U = ca.Function('se2_U', [e], [U], ['e'], ['U'])
    f_U_inv = ca.Function('se2_U_inv', [e], [U_inv], ['e'], ['U_inv'])
    f_U_inv_series = ca.Function('se2_U_inv_series', [e], [U_inv_series], ['e'], ['U_inv'])

    assert np.max(f_U([0.1, 0.2, 3])@f_U_inv([0.1, 0.2, 3]) - np.eye(3)) < 1e-10

    functions = [f_U, f_U_inv, se2_error]
    return { f.name(): f for f in functions }


def derive_rover2d_estimator():
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

    eqs = {"predict": f_predict}
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
    parser = argparse.ArgumentParser()
    parser.add_argument('dest_dir')
    args = parser.parse_args()

    print("generating casadi equations in {:s}".format(args.dest_dir))

    #rover_plan()
    #plt.show()
    #test_bezier()

    print("generating casadi equations")
    # derivate casadi functions
    eqs = {}
    eqs.update(derive_bezier6())
    eqs.update(derive_rover())
    eqs.update(derive_se2())
    eqs.update(derive_rover2d_estimator())

    for name, eq in eqs.items():
        print('eq: ', name)

    generate_code(eqs, filename="melm.c", dest_dir=args.dest_dir)
    print("complete")
