#!/usr/bin/env python
# Initial Date: November 2019
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
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

from layout import layout

'''
Summary of Class layout:
This class requires a ros package 'Sparkle'

Attributes:
    1. R: Radius of the Circle
    2. theta: angular separation of two consecutive vehicle on the circle

    and all attributes of super class

Functions:
    1. __init__(circumference, n_vehicles): basically a constructor
'''
class circle(layout, object):
    '''
    `circle`: Simulation of Connected-and-Intelligent Vehicle on Circular Trajectory.
        Inherits from its superclass `layout`.

    Parameters
    -------------
    kwargs
            variable keyword arguments

            circumference: `double`
                Circumference of circle in meters.
            
                Default Value: 230 m
            n_vehicles: `integer`
                Number of vehicles on circular circuit in simulation

                Default Value: 1

    Attributes
    ------------
    theta: `double`
        Angular Separation of Cars on Circular Trajectory

    R: `double`
        Radius of Circular Trajectory

    L: `double`
        Wheelbase of each car
    
    car_to_bumper:  `double`
        Length of car from bumper to bumper

                
    See Also
    ---------
    layout: superclass of `circle`

    '''
    
    def __init__(self, **kwargs):

        # Default args
        self.circumference = 230.0
        self.n_vehicles = 1

        try:
            self.circumference = kwargs["circumference"]
        except KeyError as e:
            pass
        try:
            self.n_vehicles = kwargs["n_vehicles"]
        except KeyError as e:
            pass    
        
        try:
            self.include_laser = kwargs["include_laser"]
        except KeyError as e:
            pass

        """Generate coordinate on x-axis to place `n_vehicles`"""
        r = self.circumference/(2*np.pi) #Calculate the radius of the circle
        print('************ Radius of the circle is {} ************'.format(r))
        self.R = r

        self.car_to_bumper = 4.52
        self.L = 2.70002 #wheelbase

        theta = (2*3.14159265359)/self.n_vehicles #calculate the minimum theta in polar coordinates for placing cars on the circle
        self.theta = theta

        print('Theta:{} radian.'.format(theta))

        self.const_angle = np.arctan(self.L/self.R)
        print('Constant Steering Angle:={}'.format(self.const_angle))

        X = [] # X-coordinates of cars on  the circle
        Y = [] # Y-coordinate of cars  on the circle
        Yaw = [] #Yaw of cars placed on the circle, with respect to the world frame

        # Calculate, X, Y and Yaw of each cars on the circle. They are assumed to be placed at a equal separation.
        for i in range(0, self.n_vehicles):
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

        super(circle, self).__init__(self.n_vehicles, X, Y, Yaw, max_update_rate =  kwargs["max_update_rate"] , time_step = kwargs["time_step"], update_rate = kwargs["update_rate"], log_time = kwargs["log_time"], include_laser=kwargs["include_laser"], description = kwargs["description"])

    ## We will define some simulation sequence that can be called without fuss
    def simulate(self,logdir):
        '''
        Class method `simulate` specifies state-based model for simulation of vehicles on circular trajectory.

        - `super.create()` -> `super.spawn()` -> `super.control()` -> `super.rviz()`

        - Simulation runs for specified `log_time`

        - Retrieves  the bag file recorded.

        Parameters
        ------------
        logdir: `string`
            Directory/Path where bag files and other data files recording simulation statistics will be saved.

        Returns
        -----------
        `string`:
            The full path of the bag file recorded for the simulation.

        '''
        self.create()

        # spawn all the vehicles
        self.spawn() # spawn() calls relevant functions to start roscore, gzserver, gzclient and rviz.
        time.sleep(4)

        radius =  self.R
       
        angle =  self.const_angle
        
        #Car's length, value reported here is the length of bounding box along the longitudinal direction of the car
       
        # initial_distance =    (self.circumference - self.n_vehicles*self.car_to_bumper )/ (self.n_vehicles - 1)

        self.control(leader_vel = 3.5, str_angle = angle, follower_vel_method="uniform", logdir=logdir)
        #self.control(leader_vel=3.5, str_angle=angle, follower_vel_method="ovftl", initial_distance =initial_distance )

        self.rviz()

        # Start Rosbag record for 60 seconds
        time.sleep(self.log_time)

        print('Simulation complete, time to terminate.')
        self.destroy(signal.SIGINT)
        
        bag = self.latesbag()

        return bag
