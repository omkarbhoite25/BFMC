import numpy as np


class ModelCar:
    # Vehicle driving parameters
    MIN_SPEED = -0.5                    # minimum speed [m/s]
    MAX_SPEED = 1.5                     # maximum speed [m/s]
    MAX_ACCEL = 0.5                     # maximum accel [m/ss]
    MAX_STEER = np.deg2rad(30.0)        # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(40.0)       # maximum steering speed [rad/s]

    # Vehicle parameters
    LENGTH = 0.45  			            # car body length [m]
    WIDTH = 0.18   			            # car body width [m]
    BACKTOWHEEL = 0.10  		        # distance of the wheel and the car body [m]
    WHEEL_LEN = 0.03  			        # wheel raduis [m]
    WHEEL_WIDTH = 0.03  		        # wheel thickness [m]
    TREAD = 0.09  			            # horizantal distance between the imaginary line through the center of the car to where the wheel is on the body (usually width/2) [m]
    WB = 0.26  			                # distance between the two wheels centers on one side (front and back wheel) [m]


class Constants:
    TARGET_SPEED = ModelCar.MAX_SPEED * 1        # target speed [m/s]
    STOP_SPEED = ModelCar.MAX_SPEED * 0.5        # stop speed [m/s]
    GOAL_DIS = 1.5                               # goal distance [m]

    MAX_TIME = 500.0                             # max simulation time
    N_IND_SEARCH = 10                            # Search index number
