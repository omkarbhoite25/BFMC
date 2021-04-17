#!/usr/bin/python
import rospy
import numpy as np
import time
import cv2
import os
from bfmclib.camera_s import CameraHandler
import matplotlib.pyplot as plt
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.LaneKeeping import LaneKeeping as lk
from bfmclib.Mask import Mask
from std_msgs.msg import Int16

class Lanes(object):

  def __init__(self):
    self.pub = rospy.Publisher('lanes', Int16, queue_size=1)
    rospy.init_node('Lane_Detector', anonymous=True)
    cam = CameraHandler()
    while 1:
      frame = cam.getImage()
      cv2.imshow("clean",frame)
      # if len(frame.shape) > 2 and frame.shape[2] == 3:
      #   img_dims = frame[:,:,0].shape
      #   mask = Mask(4, img_dims)
      #   mask.set_polygon(np.array([[0,460], [640,460], [546,155], [78, 155]]))
      #   processed_img = hf.image_processing(frame)
      #   masked_img = mask.apply_to_img(processed_img)
      #   # cv2.imshow("Masked Image", masked_img)
      #   lane_lines = hf.detect_lane(masked_img)
      #   line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
      #   line_segments_img = hf.get_hough_img(frame, line_segments)
      #   #cv2.imshow("Line Segments", line_segments_img)
      #   two_lines_specification_img = hf.line_tester(frame, line_segments)
      #   steer_angle = lk.get_steering_angle(lane_lines, frame)
      #   cv2.putText(two_lines_specification_img, "detected " + str(steer_angle) + " deg", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
      #   tar = hf.display_heading_line(two_lines_specification_img, steer_angle)
      #   cv2.imshow("lanes_node", tar)
      #   self.pub.publish(steer_angle)
      key = cv2.waitKey(1)
      
      if key == ord('q'):
        cv2.destroyAllWindows()
        break

if __name__ == "__main__":
  Lanes()

	
	#print(lane_lines)
	# lane = mf()
	# lanekeep = lane.lane_keeping(masked_img,frame)
	#cv2.imshow("lane",lane_lines) 
