#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars along x-axis. """


import time
import signal

import signal
import numpy as np
import glob
import os
from .layout import layout

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
class lane(layout, object):
    '''
    `lane`: Simulation of Connected-and-Intelligent Vehicle on a straight line.
        Inherits from its superclass `layout`.

    Parameters
    -------------
    kwargs
            variable keyword arguments

    n_vehicles: `integer`
        Keyword argument. 
        
        Number of vehicles on lane  in simulation
        
        Default Value: 1

    Attributes
    ------------
    theta: `double`
        Angular Separation of Cars on Circular Trajectory

    L: `double`
        Wheelbase of each car
    
    car_to_bumper:  `double`
        Length of car from bumper to bumper

                
    See Also
    ---------
    layout: superclass of `lane`

    '''
    def __init__(self, **kwargs):

        # default args
        self.n_vehicles = kwargs.get("n_vehicles", 1)
        self.vehicle_spacing = kwargs.get("vehicle_spacing", 15.0)
        self.include_laser = kwargs.get("include_laser", False)
        self.car_to_bumper = 4.52

        # set spatial positions of vehicles to be spawned
        Y = [0.0]
        X = [100.0]*self.n_vehicles
        Yaw =  [1.57079632679]*self.n_vehicles
        for i in range(1, int(self.n_vehicles/2)+1):
            Y .append(self.vehicle_spacing*i)
        for i in range(1, int(self.n_vehicles) - int(self.n_vehicles/2)):
            Y .append(-1.0*self.vehicle_spacing*i)
        Y.sort(reverse=True)

        super(lane, self).__init__(self.n_vehicles, X, Y, Yaw, max_update_rate =  kwargs["max_update_rate"] , time_step = kwargs["time_step"], update_rate = kwargs["update_rate"], log_time = kwargs["log_time"], include_laser=kwargs["include_laser"], description = kwargs["description"])

    def simulate(self, leader_vel, logdir, **kwargs):
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
        self.spawn() # spawn calls relevant functions to start roscore, gzclient, and rviz
        time.sleep(4)
        initial_distance =    self.vehicle_spacing -  self.car_to_bumper 
        control_method = kwargs.get("control_method", "ovftl")
        # self.control(leader_vel= leader_vel, str_angle = 0.0, control_method = "uniform" ,logdir=logdir)
        self.control(leader_vel=leader_vel, str_angle=0.0 , control_method=control_method, initial_distance =initial_distance , logdir = logdir, log=False)
        self.rviz(self.package_path + "/config/magna.rviz")
        # start the rosbag record for 60 seconds
        time.sleep(self.log_time)
        print('Simulation complete, time to terminate')
        self.destroy(signal.SIGINT)
        return self.bagfile



        