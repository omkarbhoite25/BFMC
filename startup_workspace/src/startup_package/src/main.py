#!/usr/bin/python

import os
import sys
import math
import time
import copy
import argparse
import numpy as np
import matplotlib.pyplot as plt
import json

import Utils as utils
import Simulation as simulation
import MPC as MPC

from GraphMLReader import GraphMLReader
from CubicSpline import Spline2D
from Constants import ModelCar, Constants
from Utils import State, Track, MpcIntermediates

from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.controller_p import Controller
from bfmclib.camera_s import CameraHandler
import rospy

from std_msgs.msg import String, Int16


last_sign_detected = ""
def callback(data):
    global last_sign_detected, car_states
    if data.data != last_sign_detected:
        #do stuff...
        if data.data in ["Stop", "Cross_walk"]:
            car_states.insert(0 if len(car_states) < 1 else 1, data.data)
        rospy.loginfo(rospy.get_caller_id() + "I see a %s", data.data)
    last_sign_detected = data.data

lane_angle = 0
def callback_lanes(data):
    global lane_angle
    lane_angle = data.data

def callback_navigator(data):
    global car_states
    car_states.extend(json.loads(data.data))

car_states = []

def main():
    global car_states
    # cam = CameraHandler()

    # Initialize ros and bfmclib
    rospy.init_node('localization_node', anonymous=True)
    rospy.Subscriber("signs", String, callback)
    rospy.Subscriber("navigator", String, callback_navigator)
    rospy.Subscriber("lanes", Int16, callback_lanes)
    
    
    car = Controller()
    car.stop(0.0)
    print("Car Controller loaded")

    print("DRIVING!!!!")
    gps = Gps()
    bno = BNO055()
    # print("bno",bno.getYaw())
    while rospy.get_rostime() == 0:
        time.sleep(0.5)
    print("Time synced")
    while bno.getYaw() == None:
        time.sleep(0.5)
    print("bno synced")
    # rospy.spin()
    driving_loop(car, bno)

def driving_loop(car, bno):
    global lane_angle, car_states
    # while True:
    #     car.drive(0.5, lane_angle)
    #     rospy.sleep(0.29)
    while True:
        while len(car_states) > 0:
            state = car_states.pop(0)
            if state == "Stop":
                car.stop(0)
                rospy.sleep(2)
            elif state == "Cross_walk":
                car.stop(0)
                rospy.sleep(5)
            else:
                steer =  math.degrees(state['yaw']+bno.getYaw())
                if lane_angle > 0:
                    steer = min(steer, lane_angle)
                elif lane_angle < 0:
                    steer = max(steer, lane_angle)
                car.drive(state['v'], steer)
                rospy.sleep(0.29)
        car.stop(0.0)
        rospy.sleep(0.29)

if __name__ == '__main__':
    main()