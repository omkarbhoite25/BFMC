import math
import copy
import numpy as np
import matplotlib.pyplot as plt

from CubicSpline import Spline2D


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def serarch_index(cx, cy, x, y):
    dx = [icx - x for icx in cx]
    dy = [icy - y for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    return ind


def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]
        move_direction = math.atan2(dy, dx)         # slope

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:                        
            speed_profile[i] = - target_speed       # speed for moving forward    
        else:
            speed_profile[i] = target_speed         # speed for moving reverse

    speed_profile[-1] = 0.0                         # stop 

    return speed_profile


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def calc_nearest_index(state, cx, cy, cyaw, pind, n_ind_search):

    dx = [state.x - icx for icx in cx[pind:(pind + n_ind_search)]]
    dy = [state.y - icy for icy in cy[pind:(pind + n_ind_search)]]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind) + pind
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


class MpcIntermediates:
    """
    Store MPC decision parmeters
    """

    def __init__(self, cx, cy, odelta=None, oa=None, ox=None, oy=None, xref=None, target_ind=None):
        self.cx = cx
        self.cy = cy
        self.odelta = odelta
        self.oa = oa
        self.ox = ox
        self.oy = oy
        self.xref = xref
        self.target_ind = target_ind

    def checkpoint(self):
        return copy.deepcopy(self)

    def update(self, cx=None, cy=None, odelta=None, oa=None, ox=None, oy=None, xref=None, target_ind=None):
        self.cx = cx if not cx is None else None
        self.cy = cy if not cy is None else None
        self.odelta = odelta if not odelta is None else None
        self.oa = oa if not oa is None else None
        self.ox = ox if not ox is None else None
        self.oy = oy if not oy is None else None
        self.xref = xref if not xref is None else None
        self.target_ind = target_ind if not target_ind is None else None


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, t=0.0, d=None, a=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.t = t
        self.d = d
        self.a = a

    def checkpoint(self):
        return copy.deepcopy(self)

    def update(self, a, delta, dt, t, modelcar):
        # input check
        if delta >= modelcar.MAX_STEER:
            delta = modelcar.MAX_STEER
        elif delta <= -modelcar.MAX_STEER:
            delta = -modelcar.MAX_STEER

        # update speed
        v = self.v + a * dt
        if v > modelcar.MAX_SPEED:
            v = modelcar.MAX_SPEED
        elif v < modelcar.MIN_SPEED:
            v = modelcar.MIN_SPEED

        # update x, y, yaw and velocity
        self.a = a
        self.d = delta
        self.t = t
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / modelcar.WB * math.tan(delta) * dt
        self.v = v


class Track():

    def __init__(self, waypoints_x, waypoints_y, dl, n_ind_search):  
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.dl = dl
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.ck = []
        self.s = []
        self.index_dict = {}

        # compute spline path
        spline = Spline2D()
        spline.calc_spline_course(waypoints_x, waypoints_y, dl)
        self.cx, self.cy, self.cyaw, self.ck, self.s = spline.get_spline_path()

        # compute index for the closest waypoints waypoints
        pind = 0
        for j in range(1, len(waypoints_x), 1):
            dx = [waypoints_x[j] - icx for icx in self.cx[pind:(pind + n_ind_search)]]
            dy = [waypoints_y[j] - icy for icy in self.cy[pind:(pind + n_ind_search)]]
            d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
            mind = min(d)
            ind = d.index(mind) + pind
            
            # print(pind, ind, ind-pind+1)
            index_key = j-1
            self.index_dict[index_key] = (pind, ind)
            pind = ind

        # last update
        index_key = j
        self.index_dict[index_key] = (pind, len(self.cx))

    def get_waypoints(self):
        return self.waypoints_x, self.waypoints_y

    def get_spline_path_all(self):
        return self.cx, self.cy, self.cyaw, self.ck, self.s

    def get_spline_path_for_node(self, node):
        i, j = self.index_dict[node]
        return self.cx[i:j], self.cy[i:j], self.cyaw[i:j], self.ck[i:j], self.s[i:j]

    def get_spline_path_from_node_to_node(self, src, dst):
        i = self.index_dict[src][0]
        j = self.index_dict[dst][1]
        return self.cx[i:j], self.cy[i:j], self.cyaw[i:j], self.ck[i:j], self.s[i:j]
