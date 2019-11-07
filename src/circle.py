#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars along x-axis. """

import roslaunch
import rospy, rosbag
import sys, math, time
import signal
import subprocess, shlex
from subprocess import call
import sys
import signal
import psutil
import numpy as np
import matlab.engine
import glob
import os

from catlaunch import catlaunch

'''
Summary of Class catlaunch:
This class requires a ros package 'Sparkle'

Attributes:
    1. R: Radius of the Circle
    2. theta: angular separation of two consecutive vehicle on the circle

    and all attributes of super class

Functions:
    1. __init__(circumference, num_of_vehicles): basically a constructor
'''
class circle(catlaunch, object):
    
    def __init__(self, circumference, num_of_vehicles):

        """Generate coordinate on x-axis to place `num_of_vehicles`"""
        r = circumference/(2*3.14159265359) #Calculate the radius of the circle
        print('************Radius of the circle is {}'.format(r))
        self.R = r

        theta = (2*3.14159265359)/num_of_vehicles #calculate the minimum theta in polar coordinates for placing cars on the circle
        self.theta = theta

        X = [] # X-coordinates of cars on  the circle
        Y = [] # Y-coordinate of cars  on the circle
        Yaw = [] #Yaw of cars placed on the circle, with respect to the world frame

        # Calculate, X, Y and Yaw of each cars on the circle. They are assumed to be placed at a equal separation.
        for i in range(0, num_of_vehicles):
            theta_i = theta*i

            if math.fabs(theta_i) < 0.000001:
                theta_i = 0.0
            x = r*math.cos(theta_i)
            if math.fabs(x) < 0.000001:
                x = 0.0
            X.append(x)

            y = r*math.sin(theta_i)
            if math.fabs(y) < 0.000001:
                y = 0.0
            Y.append(y)
            Yaw.append(theta_i + (3.14159265359/2))

            super(circle, self).__init__(num_of_vehicles, X, Y, Yaw)