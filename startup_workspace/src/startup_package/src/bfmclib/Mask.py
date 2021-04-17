#Mask Class: A class tha represents a mask for the images, and therefore the FOV of the car.

import numpy as np
import cv2
class Mask:
    #The points that represent the mask's shape, in the form of np array [x0,y0,x1,y1,x2,y2...]
    polygon = []

    #The stencil used for masking. A matrix of shape WIDTH x SHAPE x 1 (as it has no channels)
    stencil = []

    #Constructor of the Mask.
    #@args:
    #num_of_points: The number of points the mask will have.
    #input_img_sample: A sample of the input images, so the dimensions of the stencil can be determined.
    def __init__(self, num_of_points, img_dims):
        self.points = np.zeros(num_of_points * 2)
        self.stencil = np.zeros(img_dims, dtype=np.uint8)
   
    #Calculate the points for a trapezoid
    #@args:
    #img_width: The width of the image the mask will be applied on.
    #img_height: The height of the image the mask will be applied on.
    #vertical_offset: Offset of the trapezoid on the vertical axis (default offset is 0).
    #horizontal_offset: Offset of the trapezoid on the horizontal axis (default offset is 0).
    #TO-DO
    def calculate_points_trapezoid(self, img_width, img_height, vertical_offset=0, horizontal_offset=0):
        return 0

    def apply_to_img(self, input_img):
        #Fill the stencil with 1s in the position of the polygon. (So each other pixel will have a value of 0)
        cv2.fillConvexPoly(self.stencil, self.polygon, 1)
        
        #Apply the mask to the input image
        masked_img = cv2.bitwise_and(input_img[:,:], input_img[:,:], mask=self.stencil)

        #Return the masked image
        return masked_img

    #Sets the polygon to the inputted polygon.
    #@args:
    #polygon_points: The points of the polygon.
    def set_polygon(self, polygon_points):
        self.polygon = np.copy(polygon_points)



   



