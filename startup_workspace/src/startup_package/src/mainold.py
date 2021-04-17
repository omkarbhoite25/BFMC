#!/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
from bfmclib.gps_s import Gps
from bfmclib.bno055_s import BNO055
from bfmclib.camera_s import CameraHandler
from bfmclib.controller_p import Controller
from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
from bfmclib.Mask import Mask
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.LaneKeeping import LaneKeeping 
from bfmclib.MainFunctions import MainFunctions as mf

import rospy
import cv2

import numpy as np
import random

from time import sleep

# This line should be the first line in your program
rospy.init_node('main_node', anonymous=True)

cam = CameraHandler()
print("Camera loaded")

car = Controller()
print("Controller loaded")

sem = TrafficLight()
print("Traffic lights listener")

gps = Gps()
print("Gps loaded")

bno = BNO055()
print("BNO055 loaded")

# print("Sending move with speed 0.2, steering -25")
#car.drive(0.2, -25)
# sleep(5)

# print("Sending move with speed 0.2, steering 25")
#car.drive(0.2, 25)
# sleep(5)

print("Select \"Frame preview\" window and press Q to exit")

steering = 0.0
speed = 1.0

target = [random.randrange(0, 10), random.randrange(0, 10)] # a random point

while 1:
	frame = cam.getImage()
	cv2.imshow("Frame preview", frame)

	
	#print(lane_lines)
	# lane = mf()
	# lanekeep = lane.lane_keeping(masked_img,frame)
	#cv2.imshow("lane",lane_lines) 
	
	
	key = cv2.waitKey(1)
	
	
	if key == ord('q'):
		cv2.destroyAllWindows()
		break

	## this is for keyboard operation
	elif key == ord('w'):
		car.drive(0.4, 0)
	elif key == ord('s'):
		car.drive(-0.4, 0)
	elif key == ord('d'):
		car.drive(0.2, 10)
	elif key == ord('a'):
		car.drive(0.2, -10)

	###### PROCESS FRAME ###### 
	img_dims = frame[:,:,0].shape
	mask = Mask(4, img_dims)
	mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
	processed_img = hf.image_processing(frame)
	masked_img = mask.apply_to_img(processed_img)
	#cv2.imshow("Masked Image", masked_img)
	lane_lines = hf.detect_lane(masked_img)
	line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
	line_segments_img = hf.get_hough_img(frame, line_segments)
	#cv2.imshow("Line Segments", line_segments_img)
	two_lines_specification_img = hf.line_tester(frame, line_segments)
	cv2.imshow("Two Lines Specs", two_lines_specification_img)
	

	Lane= LaneKeeping()
	curr_steering_angle= Lane.steer(masked_img, lane_lines, curr_steering_angle)
	# print("curr_steering", curr_steering_angle)

print("Car stopped. \n END")
car.stop(0.0)


# #!/usr/bin/python

# import cv2
# import numpy as np
# from enum import Enum
# import imutils
# import os
# import rospy
# import matplotlib.pyplot as plt
# from time import sleep
# import base64
# import math
# import time

# from bfmclib.gps_s import Gps
# from bfmclib.bno055_s import BNO055
# from bfmclib.camera_s import CameraHandler
# from bfmclib.controller_p import Controller
# from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
# from bfmclib.objectDetection import ObjectDetection
# #from bfmclib.redisComm import RedisComm
# from bfmclib.Line import Line
# from bfmclib.Mask import Mask
# from bfmclib.HelperFunctions import HelperFunctions as hf
# from bfmclib.LaneKeeping import LaneKeeping as lk
# from bfmclib.PathPlanning import PathPlanning as pp
# from imutils.object_detection import non_max_suppression
# from bfmclib.pedestrianDetection import PedestrianDetection
# from bfmclib.MainFunctions import MainFunctions as mf

# import sys
# #sys.path.insert(0, "/home/papafotit/.local/lib/python3.6/site-packages")
# #import networkx as nx

# # This line should be the first line in your program
# rospy.init_node('main_node', anonymous=True)

# car = Controller()
# print("Controller loaded")

# cam = CameraHandler()
# print("Camera loaded")

# ############# PARAMETERS #######################
# frame = []
# masked_img = []
# yaw_init = 0.0
# yaw = 0.0
# pitch = 0.0
# roll = 0.0
# x = 0.0
# y = 0.0
# parking_flag_1 = False
# parking_flag_2 = False
# finishline_detected = False
# intersection_navigation = False
# time_done = False
# behaviour = 0
# dist_from_vehicle = 0.0
# speed = 0.0
# angle = 0.0

# ############ BEHAVIOURS ########################
# behaviours = {
#     0: {
#         "name": "FINISH",
#         "bevaviour": mf.finish
#     },
#     1: {
#         "name": "START",
#         "bevaviour": mf.start
#     },
#     2: {
#         "name": "PATH_PLANNING",
#         "bevaviour": mf.path_planning
#     },
#     3: {
#         "name": "GPS",
#         "bevaviour": mf.gps
#     },# """
# These are the libraries you need to import in your project in order to
# be able to communicate with the Gazebo simulator
# """
# from bfmclib.gps_s import Gps
# from bfmclib.bno055_s import BNO055
# from bfmclib.camera_s import CameraHandler
# from bfmclib.controller_p import Controller
# from bfmclib.trafficlight_s import TLColor, TLLabel, TrafficLight
# from bfmclib.Mask import Mask
# from bfmclib.HelperFunctions import HelperFunctions as hf
# #from bfmclib.LaneKeeping import LaneKeeping 
# from bfmclib.MainFunctions import MainFunctions as mf

# import rospy
# import cv2

# import numpy as np
# import random

# from time import sleep

# # This line should be the first line in your program
# rospy.init_node('main_node', anonymous=True)

# cam = CameraHandler()
# print("Camera loaded")

# car = Controller()
# print("Controller loaded")

# sem = TrafficLight()
# print("Traffic lights listener")

# gps = Gps()
# print("Gps loaded")

# bno = BNO055()
# print("BNO055 loaded")

# # print("Sending move with speed 0.2, steering -25")
# # car.drive(0.2, -25)
# # sleep(5)

# # print("Sending move with speed 0.2, steering 25")
# # car.drive(0.2, 25)
# # sleep(5)

# print("Select \"Frame preview\" window and press Q to exit")

# steering = 0.0
# speed = 1.0

# target = [random.randrange(0, 10), random.randrange(0, 10)] # a random point

# while 1:
# 	frame = cam.getImage()
# 	cv2.imshow("Frame preview", frame)

# 	###### PROCESS FRAME ###### 
# 	img_dims = frame[:,:,0].shape
# 	mask = Mask(4, img_dims)
# 	mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
# 	processed_img = hf.image_processing(frame)
# 	masked_img = mask.apply_to_img(processed_img)
# 	cv2.imshow("Masked Image", masked_img)
	 
	
	
# 	key = cv2.waitKey(1)
	
# 	# if gps._ox: # if the gps is initialized
# 	# 	theta = gps._wz
# 	# 	c, s = np.cos(theta), np.sin(theta)
# 	# 	R = np.array(((c, -s), (s, c)))
# 	# 	vector_1 = np.array(target)
# 	# 	vector_2 = np.dot(np.array([gps._ox, gps._oy]), R) # point with rotation
# 	# 	# print('v2', vector_2)
# 	# 	unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
# 	# 	unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
# 	# 	dot_product = np.dot(unit_vector_1, unit_vector_2)
# 	# 	angle = np.arccos(dot_product) # angle calculation, the math may be wrong  
# 	# 	car.drive(0.3, np.rad2deg(bno.getYaw() - angle))
# 	# 	dist = np.linalg.norm(vector_1-vector_2)
# 	# 	if dist < 0.5: # if we are close to the point, pick another random target
# 	# 		target = [random.randrange(0, 10), random.randrange(0, 10)]
# 	# 	print('target', target, 'pos', [gps._ox, gps._oy], 'dist', dist)
# 	if key == ord('q'):
# 		cv2.destroyAllWindows()
# 		break

# 	## this is for keyboard operation
# 	elif key == ord('w'):
# 		car.drive(1, 0)
# 	elif key == ord('s'):
# 		car.drive(-1, 0)
# 	elif key == ord('d'):
# 		car.drive(1, 20)
# 	elif key == ord('a'):
# 		car.drive(1, -20)

# print("Car stopped. \n END")
# car.stop(0.0)


#     4: {
#         "name": "SIGNAL_ACQUISITION",
#         "bevaviour": mf.signal_acquisition
#     },
#     5: {
#         "name": "LANE_KEEPING",
#         "bevaviour": mf.lane_keeping
#     },
#     6: {
#         "name": "TOTAL_STOP",
#         "bevaviour": mf.stop
#     },
#     7: {
#         "name": "REDUCE_SPEED",
#         "bevaviour": mf.reduce_speed 
#     },
#     8: {
#         "name": "PARKING",
#         "bevaviour": mf.parking
#     },
#     9: {
#         "name": "ROUNDABOUT_NAVIGATION",
#         "bevaviour": mf.roundabout_navigation 
#     },
#     10: {
#         "name": "INTERSECTION_NAVIGATION",
#         "bevaviour": mf.intersection_navigation
#     },
#     11: {
#         "name": "REROUTE",
#         "bevaviour": mf.reroute 
#     },
#     12: {
#         "name": "REVERSE",
#         "bevaviour": mf.reverse
#     },
#     13: {
#         "name": "INCREASE_SPEED",
#         "bevaviour": mf.increase_speed
#     },
#     # 14: {
#     #     "name": "OVERTAKE",
#     #     "bevaviour": mf.overtake 
#     # },
#     15: {
#         "name": "TAILING",
#         "bevaviour": mf.tailing
#     },
#     # 16: {
#     #     "name": "SIGN_DETECTION",
#     #     "bevaviour": mf.sign_detection
#     # },
#     17: {
#         "name": "PEDESTRIAN_DETECTION",
#         "bevaviour": mf.pedestrian_detection
#     },
#     18: {
#         "name": "INTERSECTION_DETECTION",
#         "bevaviour": mf.intersection_detection
#     }
# }
# # 	FINISH = 0
# # 	START = 1
# # 	PATH_PLANNING = 2
# # 	GPS = 3
# # 	SIGNAL_ACQUISITION = 4
# # 	LANE_KEEPING = 5
# # 	TOTAL_STOP = 6
# # 	REDUCE_SPEED = 7
# # 	PARKING = 8
# # 	ROUNDABOUT_NAVIGATION = 9
# # 	INTERSECTION_NAVIGATION = 10
# # 	REROUTE = 11
# # 	REVERSE = 12
# # 	INCREASE_SPEED = 13
# # 	OVERTAKE = 14
# # 	TAILING = 15
# #   SIGN_DETECTION = 16
# #   PEDESTRIAN_DETECTION = 17
# #   INTERSECTION_DETECTION = 18

# current_state = np.zeros((3,), dtype = int) # speed, angle, behaviour
# previous_state = np.zeros((3,), dtype = int)
# #sign_detected = np.zeros((2,), dtype = str)

# while True:
    
#     previous_state = current_state

#     ###### RECEIVE FRAME ######
#     frame = cam.getImage()
#     cv2.imshow("Frame preview", frame)

#     ###### PROCESS FRAME ###### 
#     img_dims = frame[:,:,0].shape
#     mask = Mask(4, img_dims)
#     mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
#     processed_img = hf.image_processing(frame)
#     masked_img = mask.apply_to_img(processed_img)

#     ###### BEHAVIOURS SETTLEMENT ######
#     behaviour = 5
    
#     sign_detected, distance, img_sign = behaviours[16]['behaviour']
#     pedestrian_detected, img_ped = behaviours[17]['behaviour']
#     intersection_detected = behaviours[18]['behaviour']
#     path, reached_Finish = behaviours[2]['behaviour']
#     x, y, yaw, pitch, roll = behaviours[3]['behaviour']
#     behaviours[4]['behaviour']
    
#     if key == ord('q') or finishline_detected is True:
#         behaviour = 0

#     if count_frames == 1:
#         behaviour = 1

#     if (sign_detected == 'Stop' and time_done is False)  or (pedestrian_detected is True and sign_detected == 'Pedestrian') or (sign_detected == 'Traffic_Light' and traffic_light_color == 'red'):
#         behaviour = 6
        
#     if sign_detected == 'Pedestrian'  or sign_detected == 'Highway_End' or bumpy_road is True or turn_detected is True or sign_detected == 'Traffic_Light'or intersection_detected is True:
#         behaviour = 7

#     if sign_detected == 'Parking' and do_parking is True:
#         behaviour = 8

#     if sign_detected == 'Roundabout' and intersection_detected is True and roundabout_navigation is True:
#         behaviour = 9

#     if intersection_detected is True and intersection_navigation is True:
#         behaviour = 10

#     if (sign_detected == 'Clossed_Road' and intersection_detected is True) or out_of_track is True:
#         behaviour = 11

#     if (sign_detected == 'No_entry' and on_turn is True) or out_of_track is True:
#         behaviour = 12

#     if sign_detected == 'Highway_Start' or ramp_detected is True:
#         behaviour = 13
    
#     if car_detected is True and do_overtake is True and dotted_line is True:
#         behaviour = 14

#     if car_detected is True  and dotted_line is False:
#    		 behaviour = 15


#     ###### BEHAVIOURS TRANSLATION TO SPEED AND SPEED ######
#     if behaviours[behaviour]['name'] is 'INTERSECTION_NAVIGATION':
#         speed, angle, intersection_navigation = behaviours[behaviour]['behaviour']
#         if intersection_navigation is False:
#             intersection_detected is False
#     elif behaviours[behaviour]['name'] is 'REROUTE':
#         pass
#     else:
#         speed, angle = behaviours[behaviour]['behaviour']

#     current_state = list([speed, angle, behaviour])

#     if current_state[0] != previous_state[0]:
#         car.drive(speed, angle)
    
#     ###### CLOSE SIMULATION ######
#     if behaviour == 0:
#         time.sleep(2)
#         cv2.destroyAllWindows()
# 	break