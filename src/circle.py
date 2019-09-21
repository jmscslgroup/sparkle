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


class catlaunch:
    def __init__(self, circumference, num_of_vehicles):
        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])
        time.sleep(2)
        self.num_of_vehicles = num_of_vehicles

        #Car's length, value reported here is the length of bounding box along the longitudinal direction of the car
        self.car_to_bumper = 4.00111
        """Generate coordinate on x-axis to place `num_of_vehicles`"""

        r = circumference/(2*3.14159265359) #Calculate the radius of the circle
        print('************Radius of the circle is {}'.format(r))
        self.R = r

        theta = (2*3.14159265359)/num_of_vehicles #calculate the minimum theta in polar coordinates for placing cars on the circle
        self.th = theta

        X = []
        Y = []
        Yaw = []

        self.name = ['magna', 'nebula', 'calista', 'proxima', 'zel',
                'zephyr', 'centauri', 'zenith', 'europa', 'elara', 'herse', 'thebe',
                'metis', 'himalia', 'kalyke', 'carpo', 'arche', 'aitne','thyone',
                'enceladus','mimas', 'tethys', 'lapetus', 'dione', 'phoebe',
                'epimetheus', 'hyperion', 'rhea', 'telesto',
                'deimos', 'phobos', 'triton', 'proteus', 'nereid', 'larissa',
                'galatea', 'despina']

        for i in range(0, num_of_vehicles):
            theta_i = theta*i
            print(theta_i)

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

        self.X = X
        self.Y = Y
        self.Yaw = Yaw

    def spawn(self):

        """Start roscore"""
        self.roscore = subprocess.Popen('roscore', stdout=subprocess.PIPE, shell=True)
        self.roscore_pid = self.roscore.pid
        time.sleep(5)
        rospy.init_node('twentytwo', anonymous=True)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #Object to launch empty world
        launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/ivory/VersionControl/catvehicle_ws/src/sparkle/launch/empty.launch"])

        #Object to spawn catvehicle in the empty world

        cli_args = []
        vel_args = []
        spawn_file = []
        vel_file =  []
        self.launchspawn = []
        self.launchvel = []
        launchfile = ['/home/ivory/VersionControl/catvehicle_ws/src/sparkle/launch/sparkle_spawn.launch']
        velfile = ['/home/ivory/VersionControl/catvehicle_ws/src/sparkle/launch/vel.launch']
        for n in range(0, self.num_of_vehicles):
            print(n)
            cli_args.append(['X:='+ str(self.X[n]), 'Y:='+ str(self.Y[n]),'yaw:='+ str(self.Yaw[n]),'robot:='+ str(self.name[n])])
            vel_args.append(['constVel:=8.0','strAng:=0.0595','R:='+ str(self.R),'robot:='+ str(self.name[n])])
            print(cli_args[n][0:])
            spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], cli_args[n])])
            vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
            self.launchspawn.append(roslaunch.parent.ROSLaunchParent(uuid, spawn_file[n]))
            self.launchvel.append(roslaunch.parent.ROSLaunchParent(uuid, vel_file[n]))

        launch.start()
        print('Empty world launched.')
        time.sleep(3)

        self.gzclient = subprocess.Popen('gzclient', stdout=subprocess.PIPE, shell=True)
        self.gzclient_pid = self.gzclient.pid

        time.sleep(2)


        for n in range(0, self.num_of_vehicles):
            print('Vehicle' + str(n) + ' spawning')
            self.launchspawn[n].start()
            time.sleep(6)
            self.launchvel[n].start()
            time.sleep(1)


    def signal_handler(self, sig, frame):
        print('You pressed Ctrl+C!')
        print('############################################')

        print('Terminating spawn launches')
        for n in range(0, self.num_of_vehicles):
            self.launchspawn[n].shutdown()
            self.launchvel[n].shutdown()

        print('Now killing gzclient')
        #kill the roscore
        self.gzclient.terminate()
        #Wait to prevent the creation of zombie processes.
        self.gzclient.wait()


        print('Now killing roscore')
        #kill the child process of roscore
        try:
            parent = psutil.Process(self.roscore_pid)
            print(parent)
        except psutil.NoSuchProcess:
            print("Parent process doesn't exist.")
            return
        children = parent.children(recursive=True)
        print(children)
        for process in children:
            print("Attempted to kill child: " + str(process))
            process.send_signal(signal.SIGTERM)

        #kill the roscore
        self.roscore.terminate()
        #Wait to prevent the creation of zombie processes.
        self.roscore.wait()

        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])

        sys.exit(0)


def main(argv):
    cl = catlaunch(260, 9)
    print(cl.X)

    cl.spawn()

    signal.signal(signal.SIGINT, cl.signal_handler)
    print('Press Ctrl+C')
    signal.pause()

if __name__ == '__main__':
    main(sys.argv[1:])

