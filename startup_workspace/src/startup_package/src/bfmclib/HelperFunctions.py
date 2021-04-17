#HelperFunctions class: A class that contains helper functions for lane detection and keeping.
import numpy as np
import cv2
import logging
import math
from Line import Line
from Mask import Mask

class HelperFunctions:

    #Returns the image with hough lines drawn on top.
    #@args:
    #img: Original image on top of which hough lines will be drawn.
    #hough_lines: The hough lines to be drawn.
    @staticmethod
    def get_hough_img(img, hough_lines, R=0, G=255, B=0):
        #Copy the original image to the hough_image
        hough_image = img.copy()

        #Check whether any lines have been detected
        if hough_lines is not None:
            
            #Draw hough lines
            if type(hough_lines) == list:
                for line in hough_lines:
                    cv2.line(hough_image, line.get_first_endpoint(), line.get_second_endpoint(), (R, G, B), 10)
            else:
                cv2.line(hough_image, hough_lines.get_first_endpoint(), hough_lines.get_second_endpoint(), (R, G, B), 10)
        else:
            pass
        
        return hough_image

    @staticmethod
    def display_heading_line(img, steering_angle, line_color=(0, 0, 255), line_width = 5):

        heading_image = np.zeros_like(img)
        height, width, _ = img.shape
            
        # Note: the steering angle of:
        # -90-0 degree: turn left
        # 0 degree: going straight
        # 0-90 degree: turn right         
        # However, the angles calculated here are in the range of 0 - 180
        #(x1,y1) are always center of the screen
        #(x2,y2) calculated with trig
        #print("Angle inside heading line: " + str(steering_angle))
        steering_angle_radian = steering_angle / 180.0 * math.pi
        
        #print("Angle inside heading line radian: " + str(steering_angle_radian))
        if(steering_angle == 0 or abs(steering_angle) == 90):
            x1 = int(width / 2)
            y1 = height
            x2 = x1
            y2 = int(height / 2)
        elif(steering_angle_radian > 0):
            x1 = int(width / 2)
            y1 = height
            x2 = int(x1 + (height / 2) / math.tan((math.pi/2) - steering_angle_radian))
            y2 = int(height / 2)
            #print("X2 is: " + str(x2))
        elif(steering_angle_radian < 0):
            x2 = int(width/2)
            x1 = int(x2 - (height/2)/math.tan((math.pi/2)+steering_angle_radian))
            y1 = int(height/2)
            y2 = height
        

        # cv2.ellipse(heading_image, (max(x2, x1), max(y1, x2)), (abs(x2-x1), abs(y2-y1)), 0, 180, 270-steering_angle, line_color, line_width)
        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(img, 0.8, heading_image, 1, 1)
        
        return heading_image
        

    
    #Returns the processed image.
    #TO-DO
    @staticmethod
    def image_processing(img):

        #Image Thresholding (thresholding values could vary)
        #ret, thresh = cv2.threshold(img, 200, 50, cv2.THRESH_BINARY)
        edges = cv2.Canny(img, 200, 400)
        return edges

    #Gets the hough lines of the input image and returns an array of Line objects
    #for the hough lines detected.
    #@args:
    #img: Input image to detect hough lines on.
    @staticmethod
    def vector_to_lines(vector_lines):
        lines = []

        if vector_lines is not None:
            for vector_line in vector_lines:
                line = Line(np.array(vector_line))
                lines.append(line)

        return lines


    #Detect line segments with hough transforms.
    #@args:
    #img: the image to detect hough lines on.
    @staticmethod
    def detect_line_segments(img):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 20  # minimal of votes
        line_segments = cv2.HoughLinesP(img, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=8, maxLineGap=4)
    
        return line_segments



    #Detects lanes in an image and transforms them into two single lines (one right and one left)
    #@args:
    #img: The processed image to detect lines on
    @staticmethod
    def detect_lane(img):
        line_segments = HelperFunctions.vector_to_lines(HelperFunctions.detect_line_segments(img))
        lane_lines = HelperFunctions.merge_lines(img, line_segments)
        
        return lane_lines
    
    #NOT USED -OBSOLETE-
    @staticmethod
    def make_line(frame, line_chars):
       height, width = frame.shape
       slope, intercept = line_chars
       #Bottom of the frame
       y1 = height
       #Make points from middle of the frame down
       y2 = int(y1 * 1 / 2)
       
       #Bound the coordinates within the frame
       x1 = max(-width, min(2*width, int((y1 - intercept) / slope)))
       x2 = max(-width, min(2*width, int((y2 - intercept) / slope)))       
       
       line = Line([[x1,y1,x2,y2]])
       return line

    #Makes an average line from the given line segments input.
    #@args:
    #line_segments: An array of Line objects, representing line segments
    @staticmethod
    def make_line_from_segments(line_segments):
       x1, x2, y1, y2 = 0, 0, 0, 0
       if len(line_segments) != 0:
        num_of_lines = len(line_segments)

        #Average through each coordinate 
        for line in line_segments:
            for x1_l, y1_l, x2_l, y2_l in line.get_endpoints():
                x1 = x1 + x1_l
                y1 = y1 + y1_l
                x2 = x2 + x2_l
                y2 = y2 + y2_l
                
        x1 = int(float(x1) / float(num_of_lines))
        x2 = int(float(x2) / float(num_of_lines))
        y1 = int(float(y1) / float(num_of_lines))
        y2 = int(float(y2) / float(num_of_lines))

       line = Line([[x1,y1,x2,y2]])
       return line

    @staticmethod
    def make_line_average(height, width, line_segments):
        x1, x2, y1, y2 = width, 0, 0, height

        if len(line_segments) != 0:

            for line in line_segments:
                for x1_l, y1_l, x2_l, y2_l in line.get_endpoints():
                    x1 = min(x1,x1_l)
                    y1 = max(y1, y1_l)
                    x2 = max(x2, x2_l)
                    y2 = min(y2, y2_l)

        line = Line([[x1,y1,x2,y2]])
        return line

    @staticmethod
    def merge_lines(frame, line_segments):
        lane_lines = []
        
        #If no line segments detected in frame, return an empty array
        if line_segments is None:
            return lane_lines
        
        #Get the height and width of the frame
        height, width = frame.shape
        #Array of lines of the left lane
        left_fit = []
        #Array of lines of the right lane
        right_fit = []
        
        #Boundary for left and right lanes
        boundary = 1.0/3.0
        #Left lane line segment should be on left 2/3 of the screen
        left_region_boundary = width * (1 - boundary)
        #Right lane line segment should be on right 2/3 of the screen
        right_region_boundary = width * boundary
        
        boundary_height = 1.0/2.0
        upper_boundary = height * boundary_height

        
        #Loop through the line segments
        for line_segment in line_segments:
            #Get the coordinates of the specific line segment
            
            coordinates = line_segment.get_endpoints()
            for x1, y1, x2, y2 in coordinates:


                #Check if the line is almost vertical
                if np.abs(x1-x2) < 0.06:
                    slope = None
                else:
                    #Get slope of line

                    slope = float(y2 - y1) / float(x2 - x1)

                    #Check if slope is too small, so line is almost horizontal
                    #
                    if math.fabs(slope) < 0.5:
                        continue

                #Restrict detection to the lower half of the frame, and make an additional check for horizontal lines (if the deltaY is bigger than 10)
                if y1 > upper_boundary and y2 > upper_boundary and np.abs(y2-y1) > 10:
                    #In each case based on slope and x coordinates, append the segment to the correct half of the frame
                    if (slope is None):
                        if x1 < left_region_boundary: #and x2 < left_region_boundary:
                            left_fit.append(line_segment)     
                        if x2 > right_region_boundary: #and x1 > right_region_boundary:
                            right_fit.append(line_segment)                         
                    elif slope < 0:
                        if x1 < left_region_boundary: #and x2 < left_region_boundary:
                            left_fit.append(line_segment)
                    elif slope > 0:
                        if x2 > right_region_boundary:# and x2 > right_region_boundary:
                            right_fit.append(line_segment)
        
                
        

        #left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            #lane_lines.append(HelperFunctions.make_line(frame, left_fit_average))
            lane_lines.append(HelperFunctions.make_line_from_segments(left_fit ))
            #lane_lines.append(HelperFunctions.make_average_line(left_fit))
        #right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            #lane_lines.append(HelperFunctions.make_line(frame, right_fit_average))
            lane_lines.append(HelperFunctions.make_line_from_segments(right_fit ))
            #lane_lines.append(HelperFunctions.make_average_line(right_fit))
        return lane_lines
    

    @staticmethod
    def horizontal_line_detector(frame, line_segments):
        lane_lines = []
        
        #If no line segments detected in frame, return an empty array
        if line_segments is None:
            return lane_lines
        
        #Get the height and width of the frame
        height, width, i = frame.shape
        #Array of lines of the left lane
        left_fit = []
        #Array of lines of the right lane
        right_fit = []
        #Array of horizontal lines
        horizontal_lines = []

        #Boundary for left and right lanes
        boundary_width = 1.0/3.0
        #Left lane line segment should be on left 2/3 of the screen
        left_region_boundary = width * (1 - boundary_width)
        #Right lane line segment should be on right 2/3 of the screen
        right_region_boundary = width * boundary_width

        boundary_height = 1.0/2.0
        upper_boundary = height * boundary_height

        intersection_boundary = height - (0.6) * height
        for line_segment in line_segments:
            coordinates = line_segment.get_endpoints()
            for x1, y1, x2, y2 in coordinates:
                #Fit the endpoints to a polynomial to get slope and intercept of line
                #fit = np.polyfit((x1,x2), (y1, y2), 1)
                
                #intercept = fit[1]
                line_width = np.abs(x1-x2)
                if line_width < 0.06:
                    slope = None
                else: 
                    slope = float(y2 - y1) / float(x2 - x1) 
                    segment2centerX = (x1 + x2) / 2.0
                    if line_width > 80:    
                        if math.fabs(slope) < 0.1 and math.fabs(segment2centerX - (width / 2.0)) < 40:
                            if max(y1,y2) > intersection_boundary: #and y2 > intersection_boundary:
                                horizontal_lines.append(line_segment)
                                continue


        
        #horizontal_line = HelperFunctions.make_line_average(height, width, horizontal_lines)
        horizontal_line = None
        if(len(horizontal_lines) > 0):
            horizontal_line = HelperFunctions.make_line_from_segments(horizontal_lines)
            hor_img = HelperFunctions.get_hough_img(frame, horizontal_line, R=255, G=0, B=0)
            cv2.imshow("Hor_line", hor_img)
        
        return horizontal_line

    #Developing method for testing where a line segment is detected (left right or if it horizontal)
    @staticmethod
    def line_tester(frame, line_segments):
        lane_lines = []
        
        #If no line segments detected in frame, return an empty array
        if line_segments is None:
            return lane_lines
        
        #Get the height and width of the frame
        height, width, _ = frame.shape
        #Array of lines of the left lane
        left_fit = []
        #Array of lines of the right lane
        right_fit = []
        #Array of horizontal lines
        horizontal_lines = []

        #Boundary for left and right lanes
        boundary_width = 1.0/2.0
        #Left lane line segment should be on left 2/3 of the screen
        left_region_boundary = width * (1 - boundary_width)
        #Right lane line segment should be on right 2/3 of the screen
        right_region_boundary = width * boundary_width

        boundary_height = 1.0/2.0
        upper_boundary = height * boundary_height
    
        for line_segment in line_segments:
            coordinates = line_segment.get_endpoints()
            for x1, y1, x2, y2 in coordinates:
                #Fit the endpoints to a polynomial to get slope and intercept of line
                #fit = np.polyfit((x1,x2), (y1, y2), 1)
                
                #intercept = fit[1]

                #Check if the line is almost vertical
                if np.abs(x1-x2) < 0.06:
                    slope = None
                else:
                    #Get slope of line

                    slope = float(y2 - y1) / float(x2 - x1)

                    #Check if slope is too small, so line is almost horizontal
                    #
                    #if math.fabs(slope) < 0.5:
                    #    continue

                
                if y1 > upper_boundary and y2 > upper_boundary and np.abs(y2-y1) > 10:
                    if (slope is None):
                        if x1 < left_region_boundary and x2 < left_region_boundary:
                            left_fit.append(line_segment)     
                        if x1 > right_region_boundary and x2 > right_region_boundary:
                            right_fit.append(line_segment)   
                        else:
                            continue                       
                    #elif slope < 0:
                    elif x1 < left_region_boundary: #and x2 < left_region_boundary:
                        left_fit.append(line_segment)
                    #elif slope != 0:
                    elif x1 > right_region_boundary: #and x1 > right_region_boundary:
                        right_fit.append(line_segment)


        right_img = HelperFunctions.get_hough_img(frame, right_fit, R=255, G=0, B=0)
        left_img = HelperFunctions.get_hough_img(right_img, left_fit)

        return left_img


#NIKOS INTERSECTION
    @staticmethod
    def distance2intersection(horizontal_line, img):
        height, width, i = img.shape
        horizontal_image = []
        found_intersection = False
        vector2intersection = []
        dist2intersection = []

        if((horizontal_line) != None):
            coordinates = horizontal_line.get_endpoints()
            for x1, y1, x2, y2 in coordinates:
                
                intersection_centerX = (x1 + x2) / 2
                intersection_centerY = (y1 + y2) / 2
                center = width / 2
                print("Inter X: ", intersection_centerX)
                print("Inter X: ", intersection_centerX)

                dist2intersection = np.sqrt((math.pow((center-intersection_centerX),2)+ math.pow((height-intersection_centerY),2)))
                # print(dist2intersection)
                vector2intersection = np.array([center, height, intersection_centerX, intersection_centerY])
                #img = frame.copy()

                cv2.line(img, (center, height), (intersection_centerX, intersection_centerY), (0, 0, 255), 10)

                cv2.imshow("DISTANCE", img)

                print(dist2intersection)
                print(vector2intersection)


                found_intersection = True

        return dist2intersection, found_intersection

    @staticmethod
    def stop_procedure(horizontal_line, frame):
        width, height, i = frame.shape
        timeToStop = False

        if(type(horizontal_line) != list):
            coordinates = horizontal_line.get_endpoints() 
            for x1, y1, x2, y2 in coordinates:
                y_average = (y1 + y2) / 2
            if y_average < height - 0.1 * height:
                timeToStop = True
        return timeToStop

    @staticmethod
    def second_timer(frame_counter, delay_sec):
        seconds_running = 0

        if (frame_counter >= 30):
            seconds_running += frame_counter // 30
            frame_counter = 0

        if (seconds_running <= delay_sec):
            print(seconds_running)
            # True because we are waiting at the intersection
            return True
        else:
            seconds_running = 0
            # False because we are NOT waiting at the intersection
            return False
        
    # for steering control, SetPoint is the desired angle and 
    # processValue is the current steering angle
    @staticmethod
    def PI(Kp, Ki, SetPoint, processValue):
        error = SetPoint - processValue
        # 30fps so t = 1 / 30
        t = 1.0 / 30.0

        # Proportional 
        P = Kp * error

        # Integral
        reset += (Ki / t) * error
        I = Ki * error + reset

        # Output Value
        outputValue = P + I
        print('P: ', P, 'I: ', I, 'Output Value: ', outputValue )
        return outputValue
