#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars. """

import roslaunch
import rospy, rosbag
import rospkg

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties


import sys, math, time
import signal
import subprocess, shlex
from subprocess import call
import psutil
import numpy as np
import matlab.engine
import glob
import datetime
import os
from functools import partial
from multiprocessing.pool import Pool
from multiprocessing import Process
import multiprocessing

'''
Summary of Class sparkle:
This class requires a ros package 'Sparkle'

Attributes:
    1. num_of_vehicles: Number of vehicles to be placed on circumference
    2. matlab_engine: To save matlab engine when matlab engine is started
    3. X: X-coordinates of all the vehicles with respect to the world frame
    4. Y: Y-coordinates of all the vehicles with respect to the world frame
    5. Yaw: yaw of all the vehicles with respect to the world frame
    6. callflag: a Dictionary of boolean to tell what functions were already called.

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

class sparkle:
    '''
    # SPARKLE

    Takes length of circumference and number of vehicles to be placed on the circle
    '''
    def __init__(self,  num_of_vehicles, X, Y, Yaw, **kwargs):
        print("Number of cpu : {}".format(multiprocessing.cpu_count()))
        # Define attributes for sparkle class
        self.num_of_vehicles = num_of_vehicles
        self.max_update_rate = 100
        self.time_step = 0.01
        self.log_time = 60.0
        self.include_laser  = "false"
        self.description = "sparkle simulation"
        
        try:
            # This will be used to set Gazebo physic properties
            self.max_update_rate = kwargs["max_update_rate"]
        except KeyError as e:
            pass

        try:
            self.time_step = kwargs["time_step"]
        except KeyError as e:
            pass

        try:
            # update rate will decide how often new updated will be published for Gazebo to change the poses in the model.
            self.update_rate =  kwargs["update_rate"]
        except KeyError as e:
            pass

        try:        
            self.log_time =  kwargs["log_time"]
        except KeyError as e:
            pass

        try:
            self.include_laser = "true" if kwargs["include_laser"] else "false"
        except KeyError as e:
            pass     
        
        try:
            self.description = kwargs["description"]
        except KeyError as e:
            pass

        # Kill any existing ros and gzserver and gzclient
        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])
        call(["pkill", "rosbag"])
        call(["pkill", "record"])
        time.sleep(1)




        # A member variable to store matlab engine object whenever need them
        self.matlab_engine = []

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        self.package_path = ''

        try:
            self.package_path = rospack.get_path('sparkle')
        except rospkg.ResourceNotFound as s:
            print("Package "+ str(s.args[0])+ " Not Found")
            raise

        # defining the names of the car to be spawned. Currently maximum of 22 cars
        self.name = ['magna', 'nebula', 'calista', 'proxima', 'zel',
                'zephyr', 'centauri', 'zenith', 'europa', 'elara', 'herse', 'thebe',
                'metis', 'himalia', 'kalyke', 'carpo', 'arche', 'aitne','thyone',
                'enceladus','mimas', 'tethys', 'lapetus', 'dione', 'phoebe',
                'epimetheus', 'hyperion', 'rhea', 'telesto', 'deimos',
                'phobos', 'triton', 'proteus', 'nereid', 'larissa','galatea', 'despina']

        # X-Coordinates of all the vehicles.
        self.X = X

        # Y-Coordinates of all the vehicles.
        self.Y = Y

        #Yaw of all the vehicles.
        self.Yaw = Yaw

        # A boolean dictionary that will be set to true if correspondong function is called
        self.callflag = {"log": False, "startROS":  False, "startGZserver": False, "visualize": False, "startVel": False}

        self.uuid = ""


    '''
        log function starts logging of the bag files

    '''
    def log(self):

        # specify rosbag record command with different flags, etc.
        command = ["rosbag "+ " record "+ " --all "+ " -o Circle_Test_n_" + str( self.num_of_vehicles) + '_updateRate_' + str(self.update_rate) +  '_max_update_rate_' + str(self.max_update_rate) + '_time_step_' + str(self.time_step) + '_logtime_' + str(self.log_time) + '_laser_' + self.include_laser+ ' --duration=' + str(self.log_time) +  ' __name:=bagrecorder']

        # Start Ros bag record
        print("Starting Rosbag record:{} ".format(command))
        self.rosbag_cmd = subprocess.Popen(command, shell=True, executable='/bin/bash')
        self.rosbagPID = self.rosbag_cmd.pid
        time.sleep(5)
        # So this is for temporarily generating a filename, that will change later when we will retrieve the bag file name
        dt_object = datetime.datetime.fromtimestamp(time.time())
        dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')

        # log the gz stats
        self.gzstatsFile ='gz_stats_' + dt + '.txt'
        self.gzstats = subprocess.Popen(["gz stats > " + self.gzstatsFile ], shell=True)
        self.gzstatsPID =   self.gzstats.pid

        # The log call to true once log is called
        self.callflag["log"] = True

    def stopLog(self):
        if self.callflag["log"]:
            print("Stopping ROSBAG Recorder.")
            self.rosbag_cmd.send_signal(subprocess.signal.SIGINT)
            #time.sleep(5+self.num_of_vehicles)
            bagkiller = subprocess.Popen(["rosnode kill /bagrecorder"], shell=True)

            ## Check of .bag.active is still being written
            print("Retrieving latest bag file")
            list_of_files1 = glob.glob('*.bag')
            list_of_files2 = glob.glob('*.bag.active')
            list_of_files = list_of_files1 + list_of_files2
            print(list_of_files)
            covertToBag = False
            if len(list_of_files) != 0:
                latest_file = max(list_of_files, key=os.path.getctime)
                if "bag.active" in latest_file[-10:]:
                    try:
                        filesize = os.path.getsize(latest_file)
                    except OSError as err:
                        print("OS error: {0}".format(err))
                        sys.exit()
                    while True:
                        try:
                            time.sleep(2)
                            newfilesize = os.path.getsize(latest_file)
                            byteswritten = newfilesize - filesize
                            print("Bag file is still being written. Byes written: {}".format(byteswritten))
                        except OSError as err:
                            print('.active bag file is gone.')
                            list_of_files1 = glob.glob('*.bag')
                            list_of_files2 = glob.glob('*.bag.active')
                            list_of_files = list_of_files1 + list_of_files2
                            latest_file_written = max(list_of_files, key=os.path.getctime)
                            if (latest_file_written[:-4] == latest_file[:-11]):
                                print("Bag file {} successfully written".format(latest_file_written))
                            else:
                                print("Something went wrong while finising  writing {}.".format(latest_file))
                            covertToBag = True
                            break
                        if byteswritten == 0:
                            break
                        else:
                            filesize = newfilesize

            if covertToBag is True:
                print('bag file saved successfully')



            stdout = bagkiller.communicate()
            print('rosnode kill: {}'.format(stdout))
            time.sleep(5+self.num_of_vehicles)

            print(bagkiller)



            #check if bag file has been killed:
            psaef = subprocess.Popen(["ps -aef | grep bagrecorder"], shell=True)
            stdout = psaef.communicate()
            print('ps -aef  STDOUT:{}'.format(stdout))

            call(["pkill", "rosbag"])
            call(["pkill", "record"])
            self.gzstats.terminate()

    """Start roscore"""
    def startROS(self):
        self.roscore = subprocess.Popen('roscore', stdout=subprocess.PIPE, shell=True)
        self.roscore_pid = self.roscore.pid
        self.callflag["startROS"] = True
        time.sleep(5)

    """ Kill ROS Core if it has started """
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
        launch = roslaunch.parent.ROSLaunchParent(self.uuid,[ self.package_path + "/launch/empty.launch"])
        launch.start()
        print('Empty world launched.')
        # The log call to true once log is called
        self.callflag["startGZserver"] = True
        time.sleep(3)

    def visualize(self):

        print("Start RVIZ")
        self.rviz = subprocess.Popen(["sleep 3; rosrun rviz rviz  -d" +self.package_path + "/config/magna.rviz"], stdout=subprocess.PIPE, shell=True)
        self.rviz_pid = self.rviz.pid
        self.callflag["visualize"] = True



    '''

    create function starts ros, create gazebo world and set desired physics
    properties such as max step size and max update rate.
    '''
    def create(self):
        # If roscore has not started yet, then start the roscore
        if not self.callflag["startROS"]:
            self.startROS()

        time.sleep(3)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Create a new ROS Node
        rospy.init_node('twentytwo', anonymous=True)
        roslaunch.configure_logging(self.uuid)

        if not self.callflag["startGZserver"]:
            time.sleep(3)
            self.startGZserver()

        self.gzclient = subprocess.Popen(["gzclient"], stdout=subprocess.PIPE, shell=True)
        self.gzclient_pid = self.gzclient.pid

        ## Now we will set the desired physics properties in Gazebo based on what is there in  **kwargs
        # ## using rosservice call

        rospy.wait_for_service('gazebo/get_physics_properties')
        rospy.wait_for_service('gazebo/set_physics_properties')


        get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physics_properties = get_physics_properties_prox()

        physics_properties.max_update_rate = self.max_update_rate
        physics_properties.time_step = self.time_step

        set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics_properties_prox(physics_properties.time_step,
                                    physics_properties.max_update_rate,
                                    physics_properties.gravity,
                                    physics_properties.ode_config)
        time.sleep(4)

        get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
        physics_properties = get_physics_properties_prox()
        print("Current  max_update_rate is {}".format( physics_properties.max_update_rate))

        if physics_properties.max_update_rate != self.max_update_rate:
            print("Max Update Rate was not set properly, terminating simulation")
            sys.exit()


    '''
    Spwans all cars and the impart velocity to them
    '''
    def spawn(self):

        #Object to spawn catvehicle in the empty world

        cli_args = []

        spawn_file = []

        self.launchspawn = []

        launchfile = [self.package_path + '/launch/sparkle_spawn.launch']

        n = 0
        # First Vehicle
        print('Vehicle Numer: {}'.format(n))
        cli_args.append(['X:='+ str(self.X[n]), 'Y:='+ str(self.Y[n]),'yaw:='+ str(self.Yaw[n]),'robot:='+ str(self.name[n]),'laser_sensor:=' +str(self.include_laser), 'updateRate:='+   str(self.update_rate)])

        print(cli_args[n][0:])
        spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], cli_args[n])])

        self.launchspawn.append(roslaunch.parent.ROSLaunchParent(self.uuid, spawn_file[n]))



        for n in range(1, self.num_of_vehicles):
            print('Vehicle Numer: {}'.format(n))
            cli_args.append(['X:='+ str(self.X[n]), 'Y:='+ str(self.Y[n]),'yaw:='+ str(self.Yaw[n]),'robot:='+ str(self.name[n]),'laser_sensor:=false', 'updateRate:='+   str(self.update_rate)])

            print(cli_args[n][0:])
            spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0], cli_args[n])])

            self.launchspawn.append(roslaunch.parent.ROSLaunchParent(self.uuid, spawn_file[n]))


        time.sleep(5)

        for n in range(0, self.num_of_vehicles):
            print('Vehicle [' + str(n) + '] spawning.')
            self.launchspawn[n].start()
            time.sleep(5)

    def applyVel(self, **kwargs):
        try:
           leader_vel = kwargs["leader_vel"]
           follower_vel_method = kwargs["follower_vel_method"]
           str_angle = kwargs["str_angle"]
        except KeyError as e:
            print("sparkle.applyVel: KeyError: {}".format(str(e)))
            raise

        vel_args = []
        vel_file =  []
        self.launchvel = []


        if follower_vel_method == "uniform":
            follower_vel = leader_vel
            n = 0
            vel_args.append(['constVel:='+str(leader_vel),'strAng:='+str(str_angle),'robot:='+ str(self.name[n])])
            velfile = [self.package_path + '/launch/vel.launch']

            vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
            self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            print('Velocity node ' + str(0) + '  started.')
            self.launchvel[0].start()

            # We will start ROSBag record immediately
            self.log()
            for n in range(1, self.num_of_vehicles):
                vel_args.append(['constVel:='+str(follower_vel), 'strAng:=' + str(str_angle),'robot:='+ str(self.name[n])])
                vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
                self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            for n in range(1, self.num_of_vehicles):
                print('Velocity node ' + str(n) + '  started.')
                self.launchvel[n].start()

            self.callflag["startVel"] = True

        elif follower_vel_method == "ovftl":
            initial_distance = kwargs["initial_distance"]
            n= 0

            print('self.name[self.num_of_vehicles - 1] : {}'.format(self.name[self.num_of_vehicles - 1]))
            vel_args.append(['this_name:='+self.name[n], 'leader_name:='+self.name[self.num_of_vehicles - 1],  'initial_distance:='+str(initial_distance)  , 'steering:='+ str(str_angle) , 'leader_x_init:='+str(self.X[self.num_of_vehicles-1]), 'leader_y_init:='+str(self.Y[self.num_of_vehicles-1]), 'this_x_init:='+str(self.X[n]),  'this_y_init:='+str(self.Y[n]), 'leader_odom_topic:=/' + self.name[self.num_of_vehicles - 1] + '/setvel',  'this_odom_topic:=/' + self.name[n] + '/setvel'])

            velfile = [self.package_path+'/launch/carfollowing.launch']

            vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
            self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            print('Velocity node ' + str(0) + '  started.')
            self.launchvel[0].start()

            # We will start ROSBag record immediately
            self.log()
            for n in range(1, self.num_of_vehicles):
                vel_args.append(['this_name:='+self.name[n], 'leader_name:='+self.name[n - 1],  'initial_distance:='+str(initial_distance)  , 'steering:='+ str(str_angle), 'leader_x_init:='+str(self.X[n-1]), 'leader_y_init:='+str(self.Y[n-1]), 'this_x_init:='+str(self.X[n]),  'this_y_init:='+str(self.Y[n]), 'leader_odom_topic:=/' + self.name[n - 1] + '/setvel',  'this_odom_topic:=/' + self.name[n] + '/setvel'])
                vel_file.append([(roslaunch.rlutil.resolve_launch_arguments(velfile)[0], vel_args[n])])
                self.launchvel.append(roslaunch.parent.ROSLaunchParent(self.uuid, vel_file[n]))

            for n in range(1, self.num_of_vehicles):
                print('Velocity node ' + str(n) + '  started.')
                self.launchvel[n].start()

            self.callflag["startVel"] = True

            time.sleep(3)
            call(["rosparam", "set", "/execute", "true"])

    def setUpdateRate(self, rate):
        self.update_rate = rate

    def setLogDuration(self, duration):
        print('ROSBag record duration will be {} seconds'.format(duration))
        self.log_time = duration

    def killSimulation(self, sig):
        time.sleep(5)
        print('You pressed Ctrl+C!')
        print('############################################')
        print('Terminating spawn launches')
        for n in range(0, self.num_of_vehicles):
            self.launchspawn[n].shutdown()
            if self.callflag["startVel"]:
                self.launchvel[n].shutdown()

        if self.callflag["visualize"]:
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

        self.stopLog()
        self.killROS()

        print("##### Simulation Terminated #####")

        #sys.exit(0)

    def getLatestBag(self):
        print("Retrieving latest bag file")
        if self.callflag["log"]:
            list_of_files1 = glob.glob('*.bag')
            list_of_files2 = glob.glob('*.bag.active')
            list_of_files = list_of_files1 + list_of_files2
            print(list_of_files)
            if len(list_of_files) != 0:
                latest_file = max(list_of_files, key=os.path.getctime)

                # if bag  file didn't terminate properly then we need to reindex it.
                if "bag.active" in latest_file[-10:]:
                    reindex = subprocess.Popen(["rosbag reindex " +  latest_file],   stdout=subprocess.PIPE, shell=True)
                    stdout = reindex.communicate()[0]
                    print('reindex STDOUT:{}'.format(stdout))

                    bagfix = subprocess.Popen(["rosbag fix " +  latest_file + " " + latest_file[:-7]],   stdout=subprocess.PIPE, shell=True)
                    stdout = bagfix.communicate()[0]
                    print('bagfix STDOUT:{}'.format(stdout))
                    latest_file = latest_file[:-7]

                if "bag"not  in latest_file[-3:]:
                    print("Bag file recording unsuccessful in this sesssion")
                    sys.exit()

                print("Bag File Recorded Is: " + latest_file)


                self.bagfile = latest_file

                fileName = self.bagfile[0:-4]
                create_dir = subprocess.Popen(["mkdir -v " +  fileName],   stdout=subprocess.PIPE, shell=True)
                stdout = create_dir.communicate()
                print('mkdir STDOUT:{}'.format(stdout))
                print("Renaming GZStat log file [{}] to retain bag file information".format(self.gzstatsFile))
                move_cmd = subprocess.Popen(["mv -v  " + self.gzstatsFile + " "+  fileName+"/" + fileName + "_gzStats.txt"],   stdout=subprocess.PIPE, shell=True)
                stdout = move_cmd.communicate()
                print('mv STDOUT:{}'.format(stdout))
                self.gzstatsFile = fileName+"/" +fileName + "_gzStats.txt"
                return latest_file
            else:
                print("No compatible bag file was found.")
        else:
            print("No bag was recorded in the immediate run.")
            return None


def main(argv):

    # By default number of vehicle that will be spawned is 2 when no argument is passed
    num_of_vehicle_to_spawn = 1

    if len(argv) == 0:
        print("Default num of vehicle is 1")
    elif len(argv) == 1:
        if argv[0] == '--help':
            print("Usage: ./circle [Option] [Value]");
            print("\n")
            print("\t --help: \t Get help")
            print("\t -n [Integer]: \t Pass the integer value which is the number of vehicle to spawn.")
            return
        else:
            print("Usage: ./circle.py -n 1")
            print("Also see: ./circle.py --help")
            return
    elif len(argv) == 2:
        num_of_vehicle_to_spawn = argv[1]
        print("Num of Vehicle: ", num_of_vehicle_to_spawn)
    else:
        print("Usage: ./circle.py -n 2")
        print("Also see: ./circle.py --help")
        return

    simConfig = {"circumference": 450.0, "num_vehicle":  1, "update_rate": 25, "log_time": 120, "max_update_rate": 25, "time_step": 0.01, "laser": True, "description": "catlauch test run"}

    cl = sparkle(1, [10], [20], [0.023], **simConfig)
    print(cl.X)
    cl.create()
    cl.spawn()

    time.sleep(cl.log_time)

    print('Time to terminate')
    cl.killSimulation(signal.SIGINT)

####################################################################
###########                                         Utility Functions                                #############
####################################################################

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
      parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
      print("Parent process doesn't exist.")
      return
    children = parent.children(recursive=True)
    for process in children:
      print("Attempted to kill child: " + str(process))
      process.send_signal(sig)

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            system("rosnode kill " + str)

if __name__ == '__main__':
    main(sys.argv[1:])
