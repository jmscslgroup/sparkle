#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars along x-axis. """

import sys
import math, time
import signal
import subprocess, shlex
from subprocess import call

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
    
    def __init__(self, **kwargs):

        try:
            self.circumference = kwargs["circumference"]
            self.num_of_vehicles = kwargs["num_vehicle"]
            self.update_rate = kwargs["update_rate"]
            self.log_time = kwargs["log_time"]
        except KeyError as e:
            print("circle(): KeyError: {}".format(str(e)))
            raise


        """Generate coordinate on x-axis to place `num_of_vehicles`"""
        r = self.circumference/(2*3.14159265359) #Calculate the radius of the circle
        print('************ Radius of the circle is {} ************'.format(r))
        self.R = r

        theta = (2*3.14159265359)/self.num_of_vehicles #calculate the minimum theta in polar coordinates for placing cars on the circle
        self.theta = theta

        X = [] # X-coordinates of cars on  the circle
        Y = [] # Y-coordinate of cars  on the circle
        Yaw = [] #Yaw of cars placed on the circle, with respect to the world frame

        # Calculate, X, Y and Yaw of each cars on the circle. They are assumed to be placed at a equal separation.
        for i in range(0, self.num_of_vehicles):
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

            super(circle, self).__init__(self.num_of_vehicles, X, Y, Yaw, max_update_rate =  kwargs["max_update_rate"] , time_step = kwargs["time_step"], update_rate = kwargs["update_rate"], log_time = kwargs["log_time"], laser = kwargs["laser"], description = kwargs["description"])

    ## We will define some simulation sequence that can be called without fuss
    def start_circle_sim(self):

        self.create()

        # spawn all the vehicles
        self.spawn() # spawn() calls relevant functions to start roscore, gzserver, gzclient and rviz.
        time.sleep(4)

        # Start Rosbag record for 60 seconds
        time.sleep(self.log_time)

        print('Time to terminate')
        self.killSimulation(signal.SIGINT)
        
        bagFile = self.getLatestBag()

        return bagFile