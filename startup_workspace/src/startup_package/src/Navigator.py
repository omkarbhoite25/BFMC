#!/usr/bin/python
import rospy
import numpy as np
import time
import cv2
import os
from bfmclib.camera_s import CameraHandler
import matplotlib.pyplot as plt
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.Mask import Mask
from std_msgs.msg import String
import sys

import Utils as utils
import Simulation as simulation
import MPC as MPC

from GraphMLReader import GraphMLReader
from CubicSpline import Spline2D
from Constants import ModelCar, Constants
from Utils import State, Track, MpcIntermediates
import math
import json
import matplotlib.pyplot as plt


class Navigator(object):
  def plot_car(self, state, modelcar, truckcolor="-k",):
      x = state.x
      y = state.y
      yaw = state.yaw
      steer = state.d if state.d else 0.0

      outline = np.array([[-modelcar.BACKTOWHEEL, (modelcar.LENGTH - modelcar.BACKTOWHEEL),
                          (modelcar.LENGTH - modelcar.BACKTOWHEEL), -modelcar.BACKTOWHEEL, -modelcar.BACKTOWHEEL],
                          [modelcar.WIDTH / 2, modelcar.WIDTH / 2, - modelcar.WIDTH / 2, -modelcar.WIDTH / 2, modelcar.WIDTH / 2]
      ])
      fr_wheel = np.array([[modelcar.WHEEL_LEN, -modelcar.WHEEL_LEN, -modelcar.WHEEL_LEN, modelcar.WHEEL_LEN, modelcar.WHEEL_LEN],
                            [-modelcar.WHEEL_WIDTH - modelcar.TREAD, -modelcar.WHEEL_WIDTH - modelcar.TREAD,
                            modelcar.WHEEL_WIDTH - modelcar.TREAD, modelcar.WHEEL_WIDTH - modelcar.TREAD, -modelcar.WHEEL_WIDTH - modelcar.TREAD]
      ])

      rr_wheel = np.copy(fr_wheel)

      fl_wheel = np.copy(fr_wheel)
      fl_wheel[1, :] *= -1
      rl_wheel = np.copy(rr_wheel)
      rl_wheel[1, :] *= -1

      Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
      Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                        [-math.sin(steer), math.cos(steer)]])

      fr_wheel = (fr_wheel.T.dot(Rot2)).T
      fl_wheel = (fl_wheel.T.dot(Rot2)).T
      fr_wheel[0, :] += modelcar.WB
      fl_wheel[0, :] += modelcar.WB

      fr_wheel = (fr_wheel.T.dot(Rot1)).T
      fl_wheel = (fl_wheel.T.dot(Rot1)).T

      outline = (outline.T.dot(Rot1)).T
      rr_wheel = (rr_wheel.T.dot(Rot1)).T
      rl_wheel = (rl_wheel.T.dot(Rot1)).T

      outline[0, :] += x
      outline[1, :] += y
      fr_wheel[0, :] += x
      fr_wheel[1, :] += y
      rr_wheel[0, :] += x
      rr_wheel[1, :] += y
      fl_wheel[0, :] += x
      fl_wheel[1, :] += y
      rl_wheel[0, :] += x
      rl_wheel[1, :] += y

      plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor)
      plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), truckcolor)
      plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), truckcolor)
      plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), truckcolor)
      plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), truckcolor)
      plt.plot(x, y, "*")
   

  def drive(self, track, current_x, current_y, src, dst, n, dl, dt, prev_state=None, prev_mpc=None):
          
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


  def __init__(self):
    
    self.pub = rospy.Publisher('navigator', String, queue_size=1)
    rospy.init_node('Navigator', anonymous=True)
    self.dl = 0.07       # course tick
    self.dt = 0.2        # [s] time tick
    bfmclib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    sys.path.insert(0, bfmclib_path)
    # read graph ml files
    infile = "tracks/Competition_track.graphml"
    map_reader = GraphMLReader("/home/omkar/bfmc/BFMC_Simulator/startup_workspace/src/startup_package/src/tracks/Competition_track.graphml")
    path = map_reader.get_path_from_src_to_dest(426, 467)
    dl = self.dl
    dt = self.dt
    ax = [map_reader.get_node_coordinates(node)[0] for node in path]
    ay = [map_reader.get_node_coordinates(node)[1] for node in path]

    # compute spline path for the whole track
    track = Track(ax, ay, dl, Constants.N_IND_SEARCH)

    car_states, mpc_parameters = [], []
    prev_state, prev_mpc = None, None
    n = len(track.get_waypoints()[0]) - 1
    plt.rcParams["figure.figsize"] = (10, 10)
    img = plt.imread("/home/omkar/bfmc/BFMC_Simulator/startup_workspace/src/startup_package/src/tracks/Competition_image.jpg")
    extent = [0, 15, 0, 15]
    waypoints_x, waypoints_y = track.get_waypoints()
    cx, cy, _, _, _ = track.get_spline_path_all()
    for i in range(0, n):
        print ("From " + str(i) + " To " + str(i+1))
        curr_x, curr_y = None, None
        states, mpcs, cx, cy = self.drive(track, current_x=curr_x, current_y=curr_y, src=i, dst=i+1, n=n, dl=dl, dt=dt, prev_state=prev_state, prev_mpc=prev_mpc)
        prev_state = states[-1]
        prev_mpc = mpcs[-1]
        car_states.extend(states)
        self.pub.publish(json.dumps([{
          'v': s.v,
          'yaw': s.yaw
        } for s in states]))
        
        mpc_parameters.extend(mpcs) 
        cx.extend(cx)
        cy.extend(cy)
        for i, _ in enumerate(states):
        # start = rospy.get_time()
        # accumuate values
          x, y, v, t = [], [], [], []
          target_x, target_y = [], []
          for j in range(i+1):
              x.append(states[j].x)
              y.append(states[j].y)
              v.append(states[j].v)
              t.append(states[j].t)
              target_x.append(mpcs[j].cx[mpcs[j].target_ind])
              target_y.append(mpcs[j].cy[mpcs[j].target_ind])

          # for stopping simulation with the esc key.
          plt.cla()
          plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])

          plt.plot(waypoints_x, waypoints_y, 'or', markersize=3, label="waypoints")                                           # draw waypoints
          plt.plot(cx, cy, color="r", label="course")                                                                         # draw predicted spline path
          self.plot_car(states[i], ModelCar, truckcolor='-m')                                                                      # draw car
          plt.plot(x, y, "-y", label="trajectory", linestyle='dashed')                                                        # plot car path
          plt.plot(mpcs[i].cx[mpcs[i].target_ind], mpcs[i].cy[mpcs[i].target_ind], "og", markersize=7, label="target")        # plot target index
          # plt.plot(target_x, target_y, "xg", label="target")                                                                # plot target index
          if mpcs[i].ox is not None:                                                                                          # draw mpc predicted path and reference tracjector
              plt.plot(mpcs[i].ox, mpcs[i].oy, "-b", linewidth=3, label="MPC")
          plt.plot(mpcs[i].xref[0, :], mpcs[i].xref[1, :], "-g", linewidth=3,  label="xref")

          # plot background
          plt.imshow(img, extent=extent)

          # other parameters
          plt.gca().invert_yaxis()
          plt.legend()
          plt.axis("equal")
          plt.grid(False)
          plt.title("Time[s]:" + str(round(states[i].t, 2)) + ", speed[m/s]:" + str(round(states[i].v, 2)))
        # car.stop(0.0)
        # print(states[i].yaw)
        plt.pause(0.2)


  

if __name__ == "__main__":
  Navigator()