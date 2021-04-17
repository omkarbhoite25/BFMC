from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
from bfmclib.objectDetection import ObjectDetection
# from bfmclib.redisComm import RedisComm
from bfmclib.Line import Line
from bfmclib.Mask import Mask
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.LaneKeeping import LaneKeeping as lk
from bfmclib.PathPlanning import PathPlanning as pp
# from imutils.object_detection import non_max_suppression
from bfmclib.pedestrianDetection import PedestrianDetection

from enum import Enum
#import imutils
import os
import rospy
import cv2
import matplotlib.pyplot as plt
from time import sleep
import numpy as np
import base64
import math

import sys
#sys.path.insert(0, "/home/papafotit/.local/lib/python3.6/site-packages")
#import networkx as nx

class MainFunctions:

    def __init__(self):

        # self.RedisC = RedisComm()
        # print("Redis loaded")

        self.ObjTrack = ObjectDetection()
        print("Object_Detection loaded")

        self.car = Controller()
        print("Controller loaded")

        self.sem = TrafficLight()
        print("Traffic lights listener loaded")

        self.gps = Gps()
        print("Gps loaded")

        self.bno = BNO055()
        print("BNO055 loaded")

        # self.pedTrack = PedestrianDetection()
        # print("PedDetect initiated")

        self.initial_speed = 0.4
        self.initial_angle = 0.0
        self.min_starting_speed = 0.2
        self.x, self.y, self.yaw = 0,0,0
        self.start_yaw = self.yaw
        self.source, self.finish = "86", "134"
        self.complete_path = pp.shortest_path(self.source, self.finish)
        print(self.complete_path)
        self.path = pp.remove_central_nodes(self.complete_path)
        print(self.path)

        # Parking initial parameters
        self.theta = 30
        self.phi = -30
        self.omega = 55
        self.kappa = 55
        self.park_dist = 20
        self.min_speed = 0.2
        self.counter = 0
        self.part = [False, False, False, False, False]
        self.correction = False

    def start(self):
        self.speed = self.initial_speed
        self.angle = self.initial_angle
        return self.speed, self.angle

    def finish(self):
        self.speed = 0.0
        self.angle = 0.0
        return self.speed, self.angle

    def path_planning(self):
        self.path, self.reachedFinish = pp.update_path(self.path, self.x, self.y, self.finish)

    def gps(self):
        gps_data = self.gps.getGpsData()
        if(gps_data["coor"] != None):
            self.data = self.gps.getGpsData()
            self.x = gps_data["coor"][0].real
            self.y = gps_data["coor"][0].imag
            self.yaw = math.degrees(self.bno.getYaw())
            self.pitch = math.degrees(self.bno.getPitch())
            self.roll = math.degrees(self.bno.getRoll())
        else:
            self.x = 1

        return self.x, self.y, self.yaw, self.pitch, self.roll

    def signal_acquisition(self):
        print("signal")
        # Something here to be made, or not, we will see
    
    def lane_keeping(self,masked_img, frame):
        
        #Detect lines -START-
        self.lane_lines = hf.detect_lane(masked_img)
        #Detect lines -END- 

        self.angle = lk.lane_keeping(frame, self.lane_lines, self.speed, self.angle, masked_img)
        self.angle /= 2

        return self.speed, self.angle 

    def stop(self):
        self.speed = 0.0
        self.angle = 0.0
        return self.speed, self.angle

    def reduce_speed(self):
        self.speed /= 2
        return self.speed, self.angle

    def parking(self, yaw_init, yaw, frame, flag):
        if self.counter == 10:
            self.correction = True

        ### Check the part of the parking procedure
        if yaw == yaw_init and self.part[0] is False: #Turn right and backwards
            for i in range(0, 4):
                self.part[i] = False
            self.part[0] = True

        elif yaw >= (yaw_init + self.kappa) and self.part[0] is True: #Turn left and backwards
            for i in range(0, 4):
                self.part[i] = False
            self.part[1] = True

        elif yaw <= (yaw_init + 2) and yaw >= (yaw_init - 2) and self.part[1] is True: #Correct parking
            for i in range(0, 4):
                self.part[i] = False
            self.part[2] = True

        elif yaw <= (yaw_init + 2) and yaw >= (yaw_init - 2) and self.part[2] is True and self.correction is True: #Turn left and forward
            for i in range(0, 4):
                self.part[i] = False
            self.part[3] = True
            self.correction = False
            self.counter = 0
            
        elif yaw >= (yaw_init + self.omega) and self.part[3] is True: #Turn right until 
            for i in range(0, 4):
                self.part[i] = False
            self.part[4] = True

        elif yaw <= (yaw_init + 2) and yaw >= (yaw_init - 2) and self.part[4] is True: 
            for i in range(0, 4):
                self.part[i] = False
            self.part[0] = True
            flag = False   

        ### Calculate the speed and angle
        if self.part[0] is True:
            print("Part 1")
            self.angle = self.theta
            self.speed = -self.min_speed

        elif self.part[1] is True:
            print("Part 2")
            self.angle = self.phi
            self.speed = -self.min_speed

        elif self.part[2] is True:
            print("Part 3")
            self.angle = 0
            self.speed = 0
            self.counter += 1
        
        elif self.part[3] is True:
            print("Part 4")
            self.angle = self.phi
            self.speed = self.min_speed
        
        elif self.part[4] is True:
            print("Part 5")
            self.angle = self.theta
            self.speed = self.min_speed

        return self.speed, self.angle, flag

    def roundabout_navigation(self):
        print("roundabout")
        # Something here to be made #

    def intersection_navigation(self, intersection_navigation):
        self.angle, self.reached_target = pp.intersection_navigation(self.path, self.x, 
                                                            self.y, self.target_node, 
                                                        self.start_yaw, self.yaw, 
                                                    self.complete_path)

        if(self.reached_target): 
            intersection_navigation = False
        self.intersection_navigation = intersection_navigation
        self.speed = self.min_speed

        return self.speed, self.angle, self.intersection_navigation


    def reroute(self, direction):
        print("reroute")
        # Something here to be made #
        # self.path, self.reachedFinish = pp.update_path(self.path, self.x, self.y, self.finish)

    def reverse(self):
        self.speed = -self.min_starting_speed
        self.angle = -self.angle
        return self.speed, self.angle

    def increase_speed(self):
        self.speed *= 2
        if self.speed >= self.max_speed:
            self.speed = self.max_speed
        return self.speed, self.angle

    #def overtake(self,frame):
        #To be ready soon#
    
    def tailing(self,frame, dist_from_vehicle):
        self.speed /= (480/dist_from_vehicle)

        return self.speed, self.angle
    
    # def sign_detection(self,frame):
    #     #Image Publishing
    #     self.RedisC.publish("simulation-channel", frame)
    #     print("Frame published")

    #     #Detect Sign
	# 	message = self.RedisC.getMessage(1)
	# 	while message is None:
	# 		time.sleep(0.1)
	# 		message = self.RedisC.getMessage(1)
	# 	if message is not None:
	# 		result = self.RedisC.translate(message)
	# 		if result is not 0:
	# 			print("Data inserted for printing")
	# 			cv2.imshow("Frame after editing", result)
		
	# 			#Take label
	# 			label = self.RedisC.getMessage(2)
	# 			distance[1] = self.RedisC.getMessage(3)
	# 			print("Label is ready: ", label)
	# 			print("Distance is ready: ", distance[1])
        
    #     return label, distance, imgSign

    def pedestrian_detection(self,imgPed):
        #Pedestrian Detection (a.k.a. Cringe Doll detection)
        pedDetect = self.pedTrack.detectPedestrian(imgPed)

        return pedDetect, imgPed

    def intersection_detection(self,masked_img, frame):
        #Detect intersections and distance to them -START-
        line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
        horizontal_line = hf.horizontal_line_detector(frame, line_segments)
        found_intersection = False
        if(horizontal_line != None):
            distance, detected_hor_line = hf.distance2intersection(horizontal_line, frame)
            if distance < 80:
                found_intersection = True
                target_node = pp.find_target(path)
                start_yaw = self.yaw
                #speed = 0

        return found_intersection
        #Detect intersections and distance to them -END-