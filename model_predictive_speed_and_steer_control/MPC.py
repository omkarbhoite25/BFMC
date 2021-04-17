"""

Path tracking simulation with iterative linear model predictive control for speed and steer control

author: Atsushi Sakai (@Atsushi_twi)

"""

import cvxpy
import numpy as np
import math

import Utils as utils
from Utils import State
from Constants import ModelCar, Constants


# CONSTANTS
NX = 4                                  # x = x, y, v, yaw
NU = 2                                  # a = [accel, steer]
T = 5                                   # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])               # input cost matrix
Rd = np.diag([0.01, 1.0])               # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])       # state cost matrix
Qf = Q  # state final matrix

# iterative paramter
MAX_ITER = 3                            # Max iteration
DU_TH = 0.1                             # iteration finish param


def get_linear_model_matrix(v, phi, delta, dt):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = dt * math.cos(phi)
    A[0, 3] = - dt * v * math.sin(phi)
    A[1, 2] = dt * math.sin(phi)
    A[1, 3] = dt * v * math.cos(phi)
    A[3, 2] = dt * math.tan(delta) / ModelCar.WB

    B = np.zeros((NX, NU))
    B[2, 0] = dt
    B[3, 1] = dt * v / (ModelCar.WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = dt * v * math.sin(phi) * phi
    C[1] = - dt * v * math.cos(phi) * phi
    C[3] = - dt * v * delta / (ModelCar.WB * math.cos(delta) ** 2)

    return A, B, C


def predict_motion(x0, oa, od, xref, dt):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state.update(a=ai, delta=di, t=0, dt=dt, modelcar=ModelCar)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od, dt):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref, dt)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref, dt)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref, dt):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t], dt)
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= ModelCar.MAX_DSTEER * dt]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [x[2, :] <= ModelCar.MAX_SPEED]
    constraints += [x[2, :] >= ModelCar.MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= ModelCar.MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= ModelCar.MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = utils.get_nparray_from_matrix(x.value[0, :])
        oy = utils.get_nparray_from_matrix(x.value[1, :])
        ov = utils.get_nparray_from_matrix(x.value[2, :])
        oyaw = utils.get_nparray_from_matrix(x.value[3, :])
        oa = utils.get_nparray_from_matrix(u.value[0, :])
        odelta = utils.get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind, dt):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = utils.calc_nearest_index(state, cx, cy, cyaw, pind, n_ind_search=Constants.N_IND_SEARCH)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * dt
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref
