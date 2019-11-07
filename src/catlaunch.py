#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars. """

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

'''
Summary of Class catlaunch:
This class requires a ros package 'Sparkle'

Attributes:
    1. num_of_vehicles: Number of vehicles to be placed on circumference
    2. matlab_engine: To save matlab engine when matlab engine is started
    3. car_to_bumper: Car length
    4. X: X-coordinates of all the vehicles with respect to the world frame
    5. Y: Y-coordinates of all the vehicles with respect to the world frame
    6. Yaw: yaw of all the vehicles with respect to the world frame
    7. callflag: a Dictionary of boolean to tell what functions were already called.

Functions:

    1. __init__( num_of_vehicles, X, Y, Yaw): basically a constructor
    2. log() : function upon calling starts rosbag record
    3. startROS(): function upon calling starts roscore
    4. killROS(): function upon calling kills roscore
    5. startGZserver(): starts ROS-gazebo GZserver via a launch file
    6. spawn(): spawns the number of vehicles specified in __init__
    7. enableSystem() : enable the ROS system Sparkle
    8. startTimer(): start companion Timer
    9. killTimer(): kill the companion Timer
    10. killSimulation() : handles Ctrl-C signal to terminate all the roslaunches, call kilLROS() and kill gzclient
    11. visualize(): starts ros RVIZ for visualization
    
    main(): If you decide to directly execute this file, main () will be the entry point. Otherwise you can use this script as a class definition to instantiate an object

    A proper call sequence after object instantiation:
    Option 1:    startROS() -> spawn() -> startTimer () -> enableSystem() -> log()->killSimulation()->getLatestBag()

'''

class catlaunch:
    '''
    __init__ takes length of circumference and number of vehicles to be placed on the circle
    '''
    def __init__(self,  num_of_vehicles, X, Y, Yaw):

        # Kill any existing ros and gzserver and gzclient
        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])
        time.sleep(1)
        
        # Define attributes for catlaunch class
        self.num_of_vehicles = num_of_vehicles

        # A member variable to store matlab engine object whenever need them
        self.matlab_engine = []

        #Car's length, value reported here is the length of bounding box along the longitudinal direction of the car
        self.car_to_bumper = 4.00111

        # defining the names of the car to be spawned. Currently maximum of 22 cars
        self.name = ['magna', 'nebula', 'calista', 'proxima', 'zel',
                'zephyr', 'centauri', 'zenith', 'europa', 'elara', 'herse', 'thebe',
                'metis', 'himalia', 'kalyke', 'carpo', 'arche', 'aitne','thyone',
                'enceladus','mimas', 'tethys', 'lapetus', 'dione', 'phoebe',
                'epimetheus', 'hyperion', 'rhea', 'telesto', 'deimos',
                'phobos', 'triton', 'proteus', 'nereid', 'larissa','galatea', 'despina']

        self.X = X
        self.Y = Y
        self.Yaw = Yaw

        # A boolean dictionary that will be set to true if correspondong function is called
        self.callflag = {"log": False, "startROS":  False, "startGZserver": False, "visualize": False}

        self.uuid = ""

    '''
        log function starts logging of the bag files

    '''    
    def log(self):

        command = 'rosbag record -a -o Circle_Test_n_' + str( self.num_of_vehicles) + ' __name:=bagrecorder'
        # Start Ros bag record
        print("Starting Rosbag record: " + command)
        self.rosbag_cmd = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
        self.rosbagPID = self.rosbag_cmd.pid

        # The log call to true once log is called     
        self.callflag["log"] = True

    def stopLog(self):
        if self.callflag["log"]:
            print("Stopping ROSBAG Recorder.")
            bagkiller = subprocess.Popen('rosnode kill /bagrecorder', stdout=subprocess.PIPE, shell=True)
            kill_child_processes(self.rosbagPID)
            callret = call(["kill","-9 " + str( self.rosbagPID)])

        
    """Start roscore"""
    def startROS(self):
        self.roscore = subprocess.Popen('roscore', stdout=subprocess.PIPE, shell=True)
        self.roscore_pid = self.roscore.pid
        self.callflag["startROS"] = True
        time.sleep(5)

    """ Kill ROS Code if it has started """
    def killROS(self):

        if self.callflag["startROS"]:
            print('Killing roscore')
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
        # Unconditionally trying to Kill ROS, useful in a situation where ros master was started by someone else.
        call(["pkill", "ros"])

    def startGZserver(self):

         #Object to launch empty world
        launch = roslaunch.parent.ROSLaunchParent(self.uuid,["/home/ivory/VersionControl/catvehicle_ws/src/sparkle/launch/empty.launch"])
        launch.start()
        print('Empty world launched.')
        # The log call to true once log is called     
        self.callflag["startGZserver"] = True
        time.sleep(3)

    def visualize(self):

        print("Start RVIZ")
        self.rviz = subprocess.Popen(["rosrun rviz rviz  -d ../config/magna.rviz"], stdout=subprocess.PIPE, shell=True)
        self.rviz_pid = self.rviz.pid
        self.callflag["visualize"] = True

        

    def spawn(self):

        # If roscore has not started yet, then start the roscore
        if not self.callflag["startROS"]:
            self.startROS()

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Create a new ROS Node
        rospy.init_node('twentytwo', anonymous=True)
        roslaunch.configure_logging(self.uuid)


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
            self.launchspawn.append(roslaunch.parent.ROSLaunchParent(self.uuid, spawn_file[n]))
            self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

        if not self.callflag["startGZserver"]:
            self.startGZserver()

        self.gzclient = subprocess.Popen(["gzclient"], stdout=subprocess.PIPE, shell=True)
        self.gzclient_pid = self.gzclient.pid

        time.sleep(2)

        for n in range(0, self.num_of_vehicles):
            print('Vehicle' + str(n) + ' spawning')
            self.launchspawn[n].start()
            time.sleep(6)
            self.launchvel[n].start()
            time.sleep(1)

        if not  self.callflag["visualize"]:
            self.visualize()

    def enableSystem(self):
        print("System Enabled")
        call(["rosparam", "set", "Enable", "true"])
        enableSys = subprocess.Popen(["rosparam set Enable true"], stdout=subprocess.PIPE, shell=True)

    def startTimer(self, rate):

        print("Time Started")

        # self.timer = subprocess.Popen( ["roslaunch sparkle companion_timer.launch publish_rate:="+str(rate)], stdout=subprocess.PIPE, shell=True)
        # self.timer_pid = self.timer.pid

        roslaunch.configure_logging(self.uuid)
        
        companion_args = ['/home/ivory/VersionControl/catvehicle_ws/src/sparkle/launch/companion_timer.launch', 'publish_rate:=' + str(rate)]
        launch_args = companion_args[1:]
        
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(companion_args)[0], launch_args)]

        self.timer_launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        
        self.timer_launch.start()

    def killTimer(self):
        self.timer_launch.shutdown()
        #self.timer.terminate()
        call(["pkill", "companion_timer"])

    def killSimulation(self, sig, frame):

        print('You pressed Ctrl+C!')

        self.stopLog()
        print('############################################')

        print('Terminating spawn launches')
        for n in range(0, self.num_of_vehicles):
            self.launchspawn[n].shutdown()
            self.launchvel[n].shutdown()

        self.killTimer()

        print("Kill RVIZ")
        self.rviz.terminate()
        call(["pkill", "rviz"])

        print('Now killing gzclient')
        #kill the roscore
        self.gzclient.terminate()
        #Wait to prevent the creation of zombie processes.
        self.gzclient.wait()
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])

        #sys.exit(0)

    def getLatestBag(self):
        if self.callflag["log"]:
            list_of_files = glob.glob('./Circle_Test_n_*.bag')
            print(list_of_files)
            if len(list_of_files) != 0:
                latest_file = max(list_of_files, key=os.path.getctime)
                print("Bag File Recorded Is: " + latest_file)
                self.bagfile = latest_file
                return latest_file
        else:
            print("No bag was recorded in the immediate run.")


def main(argv):

    # By default number of vehicle that will be spawned is 2 when no argument is passed
    num_of_vehicle_to_spawn = 2

    if len(argv) == 0:
        print("Default num of vehicle is 2")
    elif len(argv) == 1:
        if argv[0] == '--help':
            print("Usage: ./circle [Option] [Value]");
            print("\n")
            print("\t --help: \t Get help")
            print("\t -n [Integer]: \t Pass the integer value which is the number of vehicle to spawn.")
            return
        else:
            print("Usage: ./circle.py -n 2")
            print("Also see: ./circle.py --help")
            return
    elif len(argv) == 2:
        num_of_vehicle_to_spawn = argv[1]
        print("Num of Vehicle: ", num_of_vehicle_to_spawn)
    else:
        print("Usage: ./circle.py -n 2")
        print("Also see: ./circle.py --help")
        return

    cl = catlaunch(260, 1)
    print(cl.X)

    cl.spawn()

    signal.signal(signal.SIGINT, cl.killSimulation)
    print('Press Ctrl+C')
    signal.pause()


####################################################################
###########                                         Utility Functions                                #############
####################################################################

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
      parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
      return
    children = parent.children(recursive=True)
    for process in children:
      process.send_signal(sig)


if __name__ == '__main__':
    main(sys.argv[1:])
