import os
import sys
import math
import time
import copy
import argparse
import numpy as np
import matplotlib.pyplot as plt

import Utils as utils
import Simulation as simulation
import MPC as MPC

from GraphMLReader import GraphMLReader
from CubicSpline import Spline2D
from Constants import ModelCar, Constants
from Utils import State, Track, MpcIntermediates


def drive(track, current_x, current_y, src, dst, n, dl, dt, prev_state=None, prev_mpc=None):
        
    # get spline path from track (current pos to the end of the track)
    cx, cy, cyaw, ck, s  = track.get_spline_path_from_node_to_node(src, n - 1)

    # # compute the secondary path from current location to the closest waypoint, get spline for secondary path, add secondary path and primary path
    # ax_t, ay_t = [current_x, cx[0]], [current_y, cy[0]]
    # spline = Spline2D()
    # spline.calc_spline_course(ax_t, ay_t, dl)
    # cx_t, cy_t, cyaw_t, ck_t, s_t = spline.get_spline_path()
    # cx = cx_t + cx
    # cy = cy_t + cy
    # cyaw = cyaw_t + cyaw
    # ck = ck_t + ck
    # s = s_t + s

    # compute speed profile
    sp = utils.calc_speed_profile(cx, cy, cyaw, Constants.TARGET_SPEED)

    # initial parameters
    if not prev_state:
        time = 0.0
        state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    else:
        time = prev_state.t
        state = State(x=prev_state.x, y=prev_state.y, yaw=prev_state.yaw, v=prev_state.v)

    odelta, oa = None, None
    if not prev_mpc:
        odelta, oa = None, None
        mpc = MpcIntermediates(cx=cx, cy=cy, odelta=odelta, oa=oa)
    else:
        odelta, oa = prev_mpc.odelta, prev_mpc.oa
        mpc = MpcIntermediates(cx=prev_mpc.cx, cy=prev_mpc.cy, odelta=odelta, oa=oa)

    # smooth yaw
    cyaw = utils.smooth_yaw(cyaw)

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    # find start index and goal index
    target_ind, _ = utils.calc_nearest_index(state, cx, cy, cyaw, 0, n_ind_search=Constants.N_IND_SEARCH)       # start node
    cx_t, cy_t, _, _, _ = track.get_spline_path_for_node(min(dst, n))
    goal_ind = utils.serarch_index(cx, cy, cx_t[0], cy_t[0])
    goal = [cx_t[0], cy_t[0]]

    # run MPC here
    isgoal = False
    states, mpcs = [], [] 

    while Constants.MAX_TIME >= time:
        state = state.checkpoint()
        mpc = mpc.checkpoint()

        xref, target_ind, dref = MPC.calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind, dt)
        x0 = [state.x, state.y, state.v, state.yaw]  # current state
        oa, odelta, ox, oy, oyaw, ov = MPC.iterative_linear_mpc_control(xref, x0, dref, oa, odelta, dt=dt)
        if odelta is not None:
            di, ai = odelta[0], oa[0]

        time = time + dt
        state.update(a=ai, delta=di, dt=dt, t=time, modelcar=ModelCar)
        mpc.update(cx=cx, cy=cy, odelta=odelta, oa=oa, ox=ox, oy=oy, xref=xref, target_ind=target_ind)
        states.append(state.checkpoint())
        mpcs.append(mpc.checkpoint())

        # check goal
        d = math.hypot(state.x - goal[0], state.y - goal[1])
        isgoal = (d <= Constants.GOAL_DIS)
        if abs(target_ind - goal_ind) >= 5:
            isgoal &= False
        if isgoal:
            print("Goal")
            break
    
    # update results
    if isgoal:
        return states, mpcs, cx, cy


def main():

    dl = 0.07       # course tick
    dt = 0.2        # [s] time tick

    parser = argparse.ArgumentParser(description="Model Predictive Controller")
    parser.add_argument("--track", help="Type of the track test/race", required=True)
    parser.add_argument("--src", help="Starting node where the car starts", required=True)
    parser.add_argument("--dst", help="Destintation node where the car stops", required=True)
    parser.add_argument("--dl", help="distance tick (dl)", default=dl)
    parser.add_argument("--dt", help="time tick (dt)", default=dt)
    parser.add_argument("--ros", help="flag to show if ros as to be import and initialized True/False", default="True")
    args = parser.parse_args()

    # Initialize ros and bfmclib
    ros_init = False
    if args.ros.upper() == "TRUE":
        try:
            import rospy
            rospy.init_node('localization_node', anonymous=True)
            bfmclib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
            sys.path.insert(0, bfmclib_path)
            from bfmclib.controller_p import Controller
            car = Controller()
            print("Car Controller loaded")
            ros_init = True
            return
        except Exception as e:
            print ("Unable to initialze and ROS and its dependicies.")
            print (e)

    # # read graph ml files
    # infile = "tracks/Competition_track.graphml" if args.track.upper() == "RACE" else "tracks/Test_track.graphml"
    # map_reader = GraphMLReader(infile)
    # path = map_reader.get_path_from_src_to_dest(args.src, args.dst)
    # dl = args.dl
    # dt = args.dt
    # ax = [map_reader.get_node_coordinates(node)[0] for node in path]
    # ay = [map_reader.get_node_coordinates(node)[1] for node in path]

    # # compute spline path for the whole track
    # track = Track(ax, ay, dl, Constants.N_IND_SEARCH)

    # car_states, mpc_parameters = [], []
    # prev_state, prev_mpc = None, None
    # n = len(track.get_waypoints()[0]) - 1
    # for i in range(0, n):
    #     print ("From " + str(i) + " To " + str(i+1))
    #     curr_x, curr_y = None, None
    #     states, mpcs, cx, cy = drive(track, current_x=curr_x, current_y=curr_y, src=i, dst=i+1, n=n, dl=dl, dt=dt, prev_state=prev_state, prev_mpc=prev_mpc)
    #     prev_state = states[-1]
    #     prev_mpc = mpcs[-1]
    #     car_states.extend(states)
    #     mpc_parameters.extend(mpcs) 
    #     cx.extend(cx)
    #     cy.extend(cy)

    # # drive the car
    # if ros_init:
    #     for s in car_states:
    #         car.drive(s.v, s.yaw)
    #         time.sleep(1)
    #         car.stop(0.0)

    # # simulation
    # plt.close("all")
    # simulation.simulate(track, car_states, mpc_parameters, ModelCar, track_type=args.track)


if __name__ == '__main__':
    main()