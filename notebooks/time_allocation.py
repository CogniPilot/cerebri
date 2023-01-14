import sympy
import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt

def find_Q(deriv, poly_deg, n_legs):
    """
    Finds the cost matrix Q
    @param deriv: for cost J, 0=position, 1=velocity, etc.
    @param poly_deg: degree of polynomial
    @n_legs: number of legs in trajectory (num. waypoints - 1)
    @return Q matrix for cost J = p^T Q p
    """
    k, l, m, n, n_c, n_l = sympy.symbols('k, l, m, n, n_c, n_l', integer=True)
    # k summation dummy variable
    # n deg of polynomial

    beta = sympy.symbols('beta')  # scaled time on leg, 0-1
    c = sympy.MatrixSymbol('c', n_c, 1)  # coefficient matrices, length is n+1, must be variable (n_c)
    T = sympy.symbols('T')  # time of leg
    P = sympy.summation(c[k, 0]*sympy.factorial(k)/sympy.factorial(k-m)*beta**(k-m)/T**m, (k, m, n))  # polynomial derivative
    P = P.subs({m: deriv, n: poly_deg}).doit()
    J = sympy.integrate(P**2, (beta, 0, 1)).doit()  # cost
    p = sympy.Matrix([c[i, 0] for i in range(poly_deg+1)])  # vector of terms
    Q = sympy.Matrix([J]).jacobian(p).jacobian(p)/2  # find Q using second derivative
    assert (p.T@Q@p)[0, 0].expand() == J  # assert hessian matches cost
    
    Ti = sympy.MatrixSymbol('T', n_l, 1)
    return sympy.diag(*[
        Q.subs(T, Ti[i]) for i in range(n_legs) ])

def find_A(deriv, poly_deg, beta, n_legs, leg, value):
    """
    Finds rows of constraint matrix for setting value of trajectory and its derivatives
    @param deriv: the derivative that you would like to set, 0=position, 1=vel etc.
    @param poly_deg: degree of polynomial
    @param beta: 0=start of leg, 1=end of leg
    @n_legs: number of legs in trajectory (num. waypoints - 1)
    @leg: current leg
    @value: value of deriv at that point
    @return A_row, b_row
    """
    k, m, n, n_c, n_l = sympy.symbols('k, m, n, n_c, n_l', integer=True)
    # k summation dummy variable
    # n deg of polynomial

    c = sympy.MatrixSymbol('c', n_c, n_l)  # coefficient matrices, length is n+1, must be variable (n_c)
    T = sympy.MatrixSymbol('T', n_l, 1)  # time of leg
    
    p = sympy.Matrix([c[i, l] for l in range(n_legs) for i in range(poly_deg+1) ])  # vector of terms

    P = sympy.summation(c[k, leg]*sympy.factorial(k)/sympy.factorial(k-m)*beta**(k-m)/T[leg]**m, (k, m, n))  # polynomial derivative
    P = P.subs({m: deriv, n: poly_deg}).doit()
    A_row = sympy.Matrix([P]).jacobian(p)
    b_row = sympy.Matrix([value])
    return A_row, b_row

def find_A_cont(deriv, poly_deg, n_legs, leg):
    """
    Finds rows of constraint matrix for continuity
    @param deriv: the derivative to enforce continuity for
    @param poly_deg: degree of polynomial
    @param beta: 0=start of leg, 1=end of leg
    @n_legs: number of legs in trajectory (num. waypoints - 1)
    @leg: current leg, enforce continuity between leg and leg + 1
    @return A_row, b_row
    """    
    k, m, n, n_c, n_l = sympy.symbols('k, m, n, n_c, n_l', integer=True)
    # k summation dummy variable
    # n deg of polynomial

    c = sympy.MatrixSymbol('c', n_c, n_l)  # coefficient matrices, length is n+1, must be variable (n_c)
    T = sympy.MatrixSymbol('T', n_l, 1)  # time of leg
    
    p = sympy.Matrix([c[i, l] for l in range(n_legs) for i in range(poly_deg+1) ])  # vector of terms

    beta0 = 1
    beta1 = 0
    P = sympy.summation(
        c[k, leg]*sympy.factorial(k)/sympy.factorial(k-m)*beta0**(k-m)/T[leg]**m
        - c[k, leg + 1]*sympy.factorial(k)/sympy.factorial(k-m)*beta1**(k-m)/T[leg+1]**m, (k, m, n))  # polynomial derivative
    P = P.subs({m: deriv, n: poly_deg}).doit()
    A_row = sympy.Matrix([P]).jacobian(p)
    b_row = sympy.Matrix([0])
    return A_row, b_row

def compute_trajectory(p, T, poly_deg, deriv=0):
    S = np.hstack([0, np.cumsum(T)])
    t = []
    x = []
    for i in range(len(T)):
        beta = np.linspace(0, 1)
        ti = T[i]*beta + S[i]
        xi = np.polyval(np.polyder(np.flip(p[i*(poly_deg+1):(i+1)*(poly_deg+1)]), deriv), beta)
        t.append(ti)
        x.append(xi)
    x = np.hstack(x)
    t = np.hstack(t)
    
    return {
        't': t,
        'x': x}

def find_cost_function(poly_deg=5, min_deriv=3, rows_free=None, n_legs=2, bc_deriv=3):
    """
    Find cost function for time allocation
    @param poly_deg: degree of polynomial
    @param min_deriv: J = integral( min_deriv(t)^2 dt ), 0=pos, 1=vel, etc.
    @param rows_free: free boundary conditions
        0 pos leg 0 start
        1 pos leg 0 end
        2 vel leg 0 start
        3 vel leg 0 end
        4 acc leg 0 start
        5 acc leg 0 end
        .. repeats for next leg
    @param bc_deriv: highest derivative of derivative boundary condition
    @param n_legs: number of legs
    """
    if rows_free is None:
        rows_free = []
        
    A_rows = []
    b_rows = []

    Q = find_Q(deriv=min_deriv, poly_deg=poly_deg, n_legs=n_legs)

    # symbolic boundary conditions
    n_l, n_d = sympy. symbols('n_l, n_d', integer=True)  # number of legs and derivatives
    x = sympy.MatrixSymbol('x', n_d, n_l)
    T = sympy.MatrixSymbol('T', n_l, 1)  # time of leg
    
    # continuity
    if False:  # enable to enforce continuity
        for m in range(bc_deriv):
            for i in range(n_legs-1):
                A_row, b_row = find_A_cont(deriv=m, poly_deg=poly_deg, n_legs=n_legs, leg0=i, leg1=i+1)
                A_rows.append(A_row)
                b_rows.append(b_row)

    # position, vel, accel, beginning and end of leg
    if True:
        for i in range(n_legs):
            for m in range(bc_deriv):
                # start
                A_row, b_row = find_A(deriv=m, poly_deg=poly_deg, beta=0, n_legs=n_legs, leg=i, value=x[m, i])
                A_rows.append(A_row)
                b_rows.append(b_row)
        
                # stop
                A_row, b_row = find_A(deriv=m, poly_deg=poly_deg, beta=1, n_legs=n_legs, leg=i, value=x[m, i+1])
                A_rows.append(A_row)
                b_rows.append(b_row)


    A = sympy.Matrix.vstack(*A_rows)

    # must be square
    if not A.shape[0] == A.shape[1]:
        raise ValueError('A must be square', A.shape)
    
    b = sympy.Matrix.vstack(*b_rows)

    I = sympy.Matrix.eye(A.shape[0])
    
    # fixed/free constraints
    rows_fixed = list(range(A.shape[0]))
    for row in rows_free:
        rows_fixed.remove(row)

    # compute permutation matrix
    rows = rows_fixed + rows_free
    C = sympy.Matrix.vstack(*[I[i, :] for i in rows])

    # find R
    A_I = A.inv()
    R = (C@A_I.T@Q@A_I@C.T)
    R.simplify()

    # split R
    n_f = len(rows_fixed) # number fixed
    n_p = len(rows_free)  # number free
    Rpp = R[n_f:, n_f:]
    Rfp = R[:n_f, n_f:]
    
    # find fixed parameters
    df = (C@b)[:n_f, 0]

    # find free parameters
    dp = -Rpp.inv()@Rfp.T@df
    
    # complete parameters vector
    d = sympy.Matrix.vstack(df, dp)
    
    # find polynomial coefficients
    p = A_I@d
    
    Ti = sympy.symbols('T_0:{:d}'.format(n_legs))

    # find optimized cost
    k = sympy.symbols('k')  # time weight
    J = ((p.T@Q@p)[0, 0]).simplify() + k*sum(Ti)

    J = J.subs(T, sympy.Matrix(Ti))
    p = p.subs(T, sympy.Matrix(Ti))
    
    return {
        'T': T,
        'f_J': sympy.lambdify([Ti, x, k], J),
        'f_p': sympy.lambdify([Ti, x, k], list(p))
    }