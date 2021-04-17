import numpy as np
import cv2
import logging
import math
from Line import Line
from Mask import Mask
from HelperFunctions import HelperFunctions as hf

class LaneKeeping:

    single_line_angle = 0
     
    @staticmethod
    def get_heading_line_with_two_lanes(lane_lines, img):
        left_x2, left_y2 = lane_lines[0].get_second_endpoint()
        right_x2, right_y2 = lane_lines[1].get_second_endpoint()
        
        height, width, _ = img.shape
        
        mid = int(width/2)
        
        x_offset = (left_x2 + right_x2) / 2.0 - mid
        y_offset = int(height / 2) 
        
        return x_offset, y_offset
    
    @staticmethod
    def get_heading_line_with_one_lane(lane_lines, img):
        x1, y1 = lane_lines[0].get_first_endpoint()
        x2, y2 = lane_lines[0].get_second_endpoint()
    
        height, width, _ = img.shape
        

        x_offset = (x2 - x1) * np.sign(y1 - y2)
        y_offset = int(height / 3.0) 
   
        return x_offset, y_offset
    
    @staticmethod
    def get_steering_angle(lane_lines, img):
        

        if len(lane_lines) == 0:
            return 0
        
        if len(lane_lines) == 1:
            x_offset, y_offset = LaneKeeping.get_heading_line_with_one_lane(lane_lines, img)
        else:
            x_offset, y_offset = LaneKeeping.get_heading_line_with_two_lanes(lane_lines, img)
        
        
        #Angle in radina to center vertical line
        angle_to_mid_radian = math.atan(float(x_offset) / float(y_offset)) 

        #Convert to degrees 
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        
        #Steering angle needed
        steering_angle = (angle_to_mid_deg )
             

        return steering_angle
    
    @staticmethod
    def steer(frame, lane_lines, curr_steering_angle):
        
        if(len(lane_lines) == 0):
            return 0
        
        new_steering_angle = LaneKeeping.get_steering_angle(lane_lines, frame)
        curr_steering_angle = LaneKeeping.stabilize_steering_angle(curr_steering_angle, new_steering_angle, len(lane_lines))
        
        if(len(lane_lines) == 1):
            curr_steering_angle = max(LaneKeeping.single_line_angle, curr_steering_angle)
            LaneKeeping.single_line_angle = curr_steering_angle
        else:
            LaneKeeping.single_line_angle = 0
        
        return curr_steering_angle
    
    @staticmethod
    def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=5):
        """
        Using last steering angle to stabilize the steering angle
        This can be improved to use last N angles, etc
        if new angle is too different from current angle, only turn by max_angle_deviation degrees
        """
        if num_of_lane_lines == 2 :
            # if both lane lines detected, then we can deviate more
            max_angle_deviation = max_angle_deviation_two_lines
        else :
            # if only one lane detected, don't deviate too much
            max_angle_deviation = max_angle_deviation_one_lane
        

        angle_deviation = new_steering_angle - curr_steering_angle
        #print("New: " + str(new_steering_angle) + " Curr: " + str(curr_steering_angle))
        #print("Deviation: " + str(angle_deviation))
        if abs(angle_deviation) > max_angle_deviation:
            if(np.sign(angle_deviation) < 0):
                stabilized_steering_angle = min(int(curr_steering_angle
                                                + max_angle_deviation * angle_deviation / abs(angle_deviation)), new_steering_angle)
            elif(np.sign(angle_deviation) > 0):
                stabilized_steering_angle = max(int(curr_steering_angle
                                                + max_angle_deviation * angle_deviation / abs(angle_deviation)), new_steering_angle)

        #    print("Add: " + str(max_angle_deviation * angle_deviation / abs(angle_deviation)))
        else:
            stabilized_steering_angle = new_steering_angle
        #print('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
        return stabilized_steering_angle

    @staticmethod
    def lane_keeping(frame, lane_lines, speed, curr_steering_angle, masked_img=None):
        

        
        #DEBUG: Show line segments detected -START-
        line_segments = hf.vector_to_lines(hf.detect_line_segments(masked_img))
        line_segments_img = hf.get_hough_img(frame, line_segments)
        cv2.imshow("Line Segments", line_segments_img)
        #DEBUG: Show line segments detected -END-

        #DEBUG: Show where each line was detected (right, left) -START-
        two_lines_specification_img = hf.line_tester(frame, line_segments)
        cv2.imshow("Two Lines Specs", two_lines_specification_img)
        #DEBUG: Show where each line was detected (right, left) -END-

        
        #Calculate steering angle -START-
        curr_steering_angle= LaneKeeping.steer(masked_img, lane_lines, curr_steering_angle)
        #Calculate steering angle -END-


        #cv2.imshow("Single Line", hough_img) #Shows the merged lines
        #cv2.imshow("Masked Image", masked_img) #Shows the masked image
        #DEBUG: Various helping windows -END-
        return curr_steering_angle