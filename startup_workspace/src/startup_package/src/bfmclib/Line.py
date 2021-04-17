#Line Class: A class tha represents a single line.

import numpy as np
import math

class Line:
    #The endpoints of the line [x0,y0,x1,y1]
    endpoints = np.zeros(4)

    #Constructor of a Line.
    #@args:
    #endpoints: The endpoints of the line.
    def __init__(self, endpoints_cartesian = []):
        self.endpoints = np.copy(endpoints_cartesian)

        
    #Returns the endpoints of the line.
    #out: np array [x0,y0,x1,y0]
    def get_endpoints(self):
        return self.endpoints

    #Returns the first endpoint of the line as a tuple.
    #out: np array [x0,y0]
    def get_first_endpoint(self):
        return (self.endpoints[0][0], self.endpoints[0][1])

    #Returns the second endpoint of the line as a tuple.
    #out: np array [x1,y1]
    def get_second_endpoint(self):
        return (self.endpoints[0][2], self.endpoints[0][3])

    #Sets the endpoints of the line.
    #@args:
    #endpoints_in: The new endpoints of the line in the form of np array [x0,y0,x1,y1]
    def set_endpoints(self, endpoints_in):
        self.endpoints = np.copy(endpoints_in)

  




