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
class catvehicle(layout, object):
    '''
    `catvehicle`: Simulation of CAT Vehicle on Circular Trajectory.
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

        self.L = 2.62 #wheelbase

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

        super(catvehicle, self).__init__(self.n_vehicles, X, Y, Yaw, max_update_rate =  kwargs["max_update_rate"] , time_step = kwargs["time_step"], update_rate = kwargs["update_rate"], log_time = kwargs["log_time"], include_laser=kwargs["include_laser"], package_name = kwargs["package_name"], description = kwargs["description"])

    def physicsengine(self):
        '''
        layout.physicsengine()

        Class method `physicsengine` starts the physics engine simulator.
        Currently, only Gazebo is supported. The plan will be to add Unreal Engine and Unity in the future.

        Sets the `layout.callflag["physicsengine"]` to `True`.
        '''
         #Object to launch empty world
        launch = roslaunch.parent.ROSLaunchParent(self.uuid,[ self.package_path + "/launch/catvehicle_empty.launch"])
        launch.start()
        print('Empty world launched.')
        # The log call to true once log is called
        self.callflag["physicsengine"] = True
        time.sleep(3)

    def spawn(self):
        '''
        `layout.spawn()` spawns the simulated vehicles in the physics world.
        '''
        
        #Object to spawn catvehicle in the empty world
        cli_args = []
        spawn_file = []
        self.launchspawn = []
        launchfile = [self.package_path + '/launch/catvehicle_spawn.launch']

        n = 0
        # First Vehicle
        print('Vehicle Numer: {}'.format(n))
        cli_args.append(['X:='+ str(self.X[n]), 'Y:='+ str(self.Y[n]),'yaw:='+ str(self.Yaw[n]),'robot:='+ str(self.name[n])])

        print(cli_args[n][0:])
        spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], cli_args[n])])

        self.launchspawn.append(roslaunch.parent.ROSLaunchParent(self.uuid, spawn_file[n]))

        for n in range(1, self.n_vehicles):
            print('Vehicle Numer: {}'.format(n))
            cli_args.append(['X:='+ str(self.X[n]), 'Y:='+ str(self.Y[n]),'yaw:='+ str(self.Yaw[n]),'robot:='+ str(self.name[n])])

            print(cli_args[n][0:])
            spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], cli_args[n])])

            self.launchspawn.append(roslaunch.parent.ROSLaunchParent(self.uuid, spawn_file[n]))

        time.sleep(5)

        for n in range(0, self.n_vehicles):
            print('Vehicle [' + str(n) + '] spawning.')
            self.launchspawn[n].start()
            time.sleep(5)

    def control(self, **kwargs):
        '''

        Class methods specifies control algorithm for imparting velocity to the car.
        The Control Algorithm can be either uniform, OVFTL (Optimal-Velocity Follow-The-Leader) Model, FollowerStopper or anything else.

        kwargs
            variable keyword arguments

            leader_vel: `double`
                Leader's Initial velocity in m/s
                
                Default Value: 3.0 m/s

            str_angle: `double`
                Initial Steering Angle of the leader car

                Default Value: 0.07 radian

            logdir: `string`
                Specifies directory/path where bag files and other statistics corresponding to this simulation will saved.

                Default Value: "./"

            control_method: "uniform"
                specifies **vehicle following algorithm** as control method.

                *"uniform"*: All vehicles in the circuit will have same velocity and steering angle

        '''

        leader_vel = 3.0
        str_angle = 0.07
        control_method = "uniform"
        logdir = "./"
        try:
           leader_vel = kwargs["leader_vel"]
        except KeyError as e:
            pass
        try:
           control_method = kwargs["control_method"]
        except KeyError as e:
            pass

        try:
           str_angle = kwargs["str_angle"]
        except KeyError as e:
            pass
        
        try:
           logdir = kwargs["logdir"]
        except KeyError as e:
            pass            
        vel_args = []
        vel_file =  []
        self.launchvel = []


        if control_method == "uniform":
            follower_vel = leader_vel
            n = 0
            vel_args.append(['vel:=0.0','strAng:='+str(str_angle),'robot:='+ str(self.name[n])])
            velfile = [self.package_path + '/launch/stepvel.launch']

            vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
            self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            print('Velocity node ' + str(0) + '  started.')
            self.launchvel[0].start()

            for n in range(1, self.n_vehicles):
                vel_args.append(['vel:=0.0', 'strAng:=' + str(str_angle),'robot:='+ str(self.name[n])])
                vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
                self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            for n in range(1, self.n_vehicles):
                print('Velocity node ' + str(n) + '  started.')
                self.launchvel[n].start()
                
            # We will start ROSBag record immediately
            self.log(logdir=logdir, prefix=self.package_name)
            
            time.sleep(10)
            
            for n in range(0, self.n_vehicles):
                rosparamset = subprocess.Popen(["rosparam set /" +self.name[n]+"/constVel " + str(leader_vel)  ],   stdout=subprocess.PIPE, shell=True)

            self.callflag["startVel"] = True



    ## We will define some simulation sequence that can be called without fuss
    def simulate(self, leader_vel, logdir):
        '''
        Class method `simulate` specifies state-based model for simulation of vehicles on circular trajectory.

        - `this.create()` -> `this.spawn()` -> `super.control()` -> `super.rviz()`

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

        self.control(leader_vel = leader_vel, str_angle = angle, follower_vel_method="uniform", logdir=logdir)
        #self.control(leader_vel=3.5, str_angle=angle, follower_vel_method="ovftl", initial_distance =initial_distance )

        self.rviz(self.package_path + "/config/magna_multi.rviz")

        # Start Rosbag record for 60 seconds
        time.sleep(self.log_time)

        print('Simulation complete, time to terminate.')
        self.destroy(signal.SIGINT)
        
        bag = self.latesbag()

        return bag

def main(argv):

    num_of_vehicle_to_spawn = 1
    if len(argv) == 0:
        print("Default number of vehicle to spawn is 1")
    elif len(argv) == 1:
        if argv[0] == '--help':
            print("Usage: ./catvehicle [Option] [Value]");
            print("\n")
            print("\t --help: \t Get help")
            print("\t -n [Integer]: \t Pass the integer value which is the number of vehicle to spawn.")
            return
        else:
            print("Usage: ./catvehicle.py -n 1")
            print("Also see: ./catvehicle.py --help")
            return
    elif len(argv) == 2:
        num_of_vehicle_to_spawn = argv[1]
        print("Num of Vehicle: ", num_of_vehicle_to_spawn)
    else:
        print("Usage: ./circle.py -n 2")
        print("Also see: ./circle.py --help")
        return