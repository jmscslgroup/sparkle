#!/usr/bin/env python
# Initial Date: September 2019
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars. """

import roslaunch
import rospy
import rospkg
import rostopic

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

import sys, time
import signal
import subprocess
from subprocess import call
import psutil
import numpy as np
import glob
import datetime
import os
import multiprocessing

from .launch import launch
from .gzstats import gzstats
from bagpy import bagreader
'''
Summary of Class `'layout'`:
This class requires a ros package 'Sparkle'



Functions:

    1. __init__( n_vehicles, X, Y, Yaw): basically a constructor
    2. log() : function upon calling starts rosbag record
    3. roscore(): function upon calling starts roscore
    4. killroscore(): function upon calling kills roscore
    5. physicsengine(): starts ROS-gazebo GZserver via a launch file
    6. spawn(): spawns the number of vehicles specified in __init__
    7. enableSystem() : enable the ROS system Sparkle
    8. startTimer(): start companion Timer
    9. killTimer(): kill the companion Timer
    10. destroy() : handles Ctrl-C signal to terminate all the roslaunches, call kilLROS() and kill gzclient
    11. rviz(): starts ros RVIZ for visualization

    main(): If you decide to directly execute this file, main () will be the entry point. Otherwise you can use this script as a class definition to instantiate an object

    A proper call sequence after object instantiation:
    Option 1:    roscore() -> spawn() -> startTimer () -> enableSystem() -> log()->destroy()->latesbag()

'''

class layout:
    '''
    `layout`: Base class for Simulation for Connected-and-Inteligent-Vehicle CPS. 
    Creates simulation layout for simulating vehicles and control.

    Parameters
    -------------
    X: `list`, `double`
        x-coordinate of vehicles's position in the world frame of reference
    Y: `list`, `double`
        y-coordinate of vehicles's position in the world frame of reference
    Yaw: `list`, `double`
        yaw of all the vehicles in the world frame of reference
    n_vehicles: `integer`
        Number of vehicles to spawn in the simulation
    kwargs
            variable keyword arguments

            max_update_rate: `double`
                Maximum Update Rate for Physics Engine (e.g. Gazebo) Simulator

                Default Value: 100 Hz
            
            time_step: `double`
                Time Step size taken by time-step solver during simulation
                
                Default Value: 0.01 seconds
            
            update_rate: `double`
                Update Rate to publish new state information by decoupled vehicle model
                
                Default Value: 20 Hz
            
            log_time: `double`
                Amount of time in seconds to capture data while running the simulation
                
                Default Value: 60.0 seconds
            
            description: `string`
                A descriptive text about the current simulation run
                Default Value: "Sparkle simulation"

    Attributes
    ---------------
    X: `list`, `double`
        x-coordinate of vehicles's position in the world frame of reference

    Y: `list`, `double`
        y-coordinate of vehicles's position in the world frame of reference
    
    Yaw: `list`, `double`
    
    n_vehicles: `double`
        Number of vehicles to spawn in the simulation
    
    max_update_rate: `double`
        Maximum Update Rate for Physics Engine (e.g. Gazebo) Simulator. 
        Default Value: 100 Hz
    
    time_step: `double`
        Time Step size taken by time-step solver during simulation
        Default Value: 0.01 seconds
    
    update_rate: `double`
        Update Rate to publish new state information by decoupled vehicle model
        Default Value: 20 Hz
    
    log_time: `double`
        Amount of time in seconds to capture data while running the simulation
        Default Value: 60.0 seconds
    
    description: `string`
        A descriptive text about the current simulation run
        Default Value: "Sparkle simulation"
    
    package_path: `string`
        Saved the ROS package path for the Sparkle Package
    
    name: `list`, `string`
        A list of mnemonics to name vehicle
    
    callflag:`dictionary`
        Dictionary to keep track of executation sequence of different class function
    
    uuid:
        A universally unique identifier (UUID)  for roslaunch
    
    logdir: `string`
        Path where ROSBags is saved

    gzstatsfile: `string`
        Filepath of simulation statistics file.    
        
    bagfile: `string`
        Filepath of the rosbag saved at the end of the simulation.

    Returns
    -----------
    `layout`
        An instance of the object `sparkle.layout`
    '''
    def __init__(self,  n_vehicles=1, X=0, Y=0, Yaw=0.0, **kwargs):
        print("Sparkle layout instance created.")
        print("Number of CPU on this machine: {}".format(multiprocessing.cpu_count()))
        
        # Define attributes for layout class
        self.n_vehicles = n_vehicles
        self.X = X # X-Coordinates of all the vehicles.
        self.Y = Y # Y-Coordinates of all the vehicles.
        self.Yaw = Yaw #Yaw of all the vehicles.
        self.max_update_rate = kwargs.get("max_update_rate", 100)
        self.time_step = kwargs.get("time_step", 0.01)
        self.update_rate = kwargs.get("update_rate", 20.0)
        self.log_time = kwargs.get("log_time", 60.0)
        self.description = kwargs.get("description", "Sparkle simulation")
        self.logdir = "./"
        self.package_name = kwargs.get("package_name", "sparkle")
        self.gzstatsfile = None
        self.bagfile = None
        print("PACKAGE NAME BEING EXECUTED IS {}".format(self.package_name))
        # Kill any existing ROS and gzserver and gzclient
        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])
        call(["pkill", "rosbag"])
        call(["pkill", "record"])
        time.sleep(1)

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        self.package_path = ''
        try:
            self.package_path = rospack.get_path(self.package_name)
        except rospkg.ResourceNotFound as s:
            print("Package "+ str(s.args[0])+ " Not Found")
            raise

        # defining the names of the car to be spawned. Currently maximum of 22 cars
        self.name = ['magna', 'nebula', 'calista', 'proxima', 'zel',  'zephyr', 'centauri', 'zenith', 'europa', 'elara', 'herse', 'thebe',
                'metis', 'himalia', 'kalyke', 'carpo', 'arche', 'aitne','thyone', 'enceladus','mimas', 'tethys', 'lapetus', 'dione', 'phoebe',
                'epimetheus', 'hyperion', 'rhea', 'telesto', 'deimos', 'phobos', 'triton', 'proteus', 'nereid', 'larissa','galatea', 'despina', 'sirius', 
                'rigel', 'vega', 'altair', 'capella', 'alcor','whirlpool', 'andromeda', 'milkyway',  'cygnus']

        # A boolean dictionary that will be set to true if correspondong function is called
        self.callflag = {"log": False, "roscore":  False, "physicsengine": False, "rviz": False, "startVel": False}
        self.uuid = ""

    def roscore(self,  uri="localhost", port=11311):
        """
        layout.roscore(uri="localhost", port=11311)
        Starts the roscore.

        Sets the `layout.callflag["roscore"]` to `True` only if the roscore is started locally by this particular simulation call.

        Parameters
        -------------
        uri: `string`
            Specifies ROS_MASTER_URI
            A valid IP address string or resolvable hostname
        port: `integer` [0-65535]
            port number where roscore will start
            A valid port ranges from 0 to 65535 but note that not all port numbers are allowed.

        Example
        ----------
        
        >>> L = layout()
        >>> L.roscore("ivory.local", 11321 )

        >>> M = layout()
        >>> remoteURI = "150.135.222.42"
        >>> port = 134
        >>> M.roscore(remoteURI, port)
        
        """
        os.environ["ROS_MASTER_URI"] = "http://"+uri+":"+str(port)

        # if there is already a ROS Core running on given URI and port, then don't start ros core

        if self.checkroscore(uri, port):
            return

        self.roscore = subprocess.Popen('roscore -p ' + str(port), stdout=subprocess.PIPE, shell=True)
        self.roscore_pid = self.roscore.pid
        self.callflag["roscore"] = True
        time.sleep(5)

    def checkroscore(self, uri="localhost", port=11311):
        '''

        Class method `checkroscore`: Checks if roscore is running

        Parameters
        -------------
        uri: `string`
            Specifies ROS_MASTER_URI
            A valid IP address string or resolvable hostname
        port: `integer` [0-65535]
            port number where roscore will start
            A valid port ranges from 0 to 65535 but note that not all port numbers are allowed.

        Returns
        ---------
        `boolean`
            Returns `True` if roscore is running otherwise returns false
        '''
        os.environ["ROS_MASTER_URI"] = "http://"+uri+":"+str(port)

        roscore_status = False
        try:
            rostopic.get_topic_class('/rosout')
            roscore_status = True
        except rostopic.ROSTopicIOException as e:
            roscore_status = False
            pass
        return roscore_status

    def log(self, logdir="./", prefix = "sparkle"):
        '''
        layout.log(logdir = "./")

        Class method `log`: Logs the data as bag files upon calling
        
        The function starts rosbag record in a subprocess and saved PID identifier.
        It also start logging gz stat metrics for post-analysis of simulation.
        
        Parameters
        -------------
        logdir: `string`
            Path/Directory where ROS Bag will be saved

        prefix: `string`
            A descriptive string to be prefixed at the beginning of bag file name.
        '''
        self.logdir = logdir

        # if logdir path doesn't exist, then create
        if not os.path.exists(self.logdir):
            os.mkdir(self.logdir)

        # specify rosbag record command with different flags, etc.
        command = ["rosbag "+ " record "+ "-a "+ " -o " + logdir + "/" + prefix + "_n_" + str( self.n_vehicles) + '_update_rate_' + str(self.update_rate) +  '_max_update_rate_' + str(self.max_update_rate) + '_time_step_' + str(self.time_step) + '_logtime_' + str(self.log_time) + ' --duration=' + str(self.log_time) +  ' __name:=bagrecorder']

        # Start Ros bag record
        print("Starting Rosbag record:{} ".format(command))
        rospy.loginfo("Rosbag record started")
        self.rosbag_cmd = subprocess.Popen(command, shell=True, executable='/bin/bash')
        self.rosbagPID = self.rosbag_cmd.pid
        time.sleep(5)
        # So this is for temporarily generating a filename, that will change later when we will retrieve the bag file name
        dt_object = datetime.datetime.fromtimestamp(time.time())
        dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')

        # log the gz stats
        self.gzstatsfile =self.logdir + '/gzstats_' + dt + '.txt'
        self.gzstats = subprocess.Popen(["gz stats > " + self.gzstatsfile ], shell=True)

        # The log call to true once log is called
        self.callflag["log"] = True
        time.sleep(10 + self.n_vehicles/2)

    def stoplog(self):
        '''
        layout.stoplog()

        Class method `stoplog`: Stop the logging of ROSBag data.
        
        The function gracefully terminates the `rosbag record` command and waits until the bag file writing is complete.
        The function also reindexes the bag file in case the rosbag record terminates prematurely.
        
        '''
        if self.callflag["log"]:
            print("Stopping ROSBAG Recorder.")
            self.rosbag_cmd.send_signal(subprocess.signal.SIGINT)
            bagkiller = subprocess.Popen(["rosnode kill /bagrecorder"], shell=True)

            ## Check of .bag.active is still being written
            print("Retrieving the latest bag file")
            os.chdir(self.logdir)
            list_of_files1 = glob.glob('*.bag')
            list_of_files2 = glob.glob('*.bag.active')
            list_of_files = list_of_files1 + list_of_files2
            bagconverted = False
            if len(list_of_files) != 0:
                latest_file = max(list_of_files, key=os.path.getctime)
                if "bag.active" in latest_file[-10:]:
                    try:
                        filesize = os.path.getsize(latest_file)
                    except OSError as err:
                        print("OS error: {0}".format(err))
                        sys.exit()
                    counter = 0
                    while True:
                        
                        try:
                            time.sleep(2)
                            newfilesize = os.path.getsize(latest_file)
                            byteswritten = newfilesize - filesize
                            if byteswritten == 0.0:
                                counter = counter + 1
                            
                            print("Bag file is still being written. Bytes written: {}".format(byteswritten))
                            
                            if counter > 10:
                                print("Something went wrong while saving the bag file, exiting. Please restart the simulation.")
                                self.killroscore()
                                sys.exit()
                        except OSError as err:
                            print('.active bag file is gone.')
                            list_of_files1 = glob.glob('*.bag')
                            list_of_files2 = glob.glob('*.bag.active')
                            list_of_files = list_of_files1 + list_of_files2
                            latest_file_written = max(list_of_files, key=os.path.getctime)
                            if (latest_file_written[:-4] == latest_file[:-11]):
                                print("Bag file {} successfully written".format(latest_file_written))
                            else:
                                print("Something went wrong while finising the  writing of {}.".format(latest_file))
                            bagconverted = True
                            break
                        if byteswritten == 0:
                            break
                        else:
                            filesize = newfilesize

            if bagconverted is True:
                print('bag file saved successfully')

            stdout = bagkiller.communicate()
            print('rosnode kill: {}'.format(stdout))
            time.sleep(5+self.n_vehicles)

            #check if bag file has been killed:
            psaef = subprocess.Popen(["ps -aef | grep bagrecorder"], shell=True)
            stdout = psaef.communicate()
            print('ps -aef  STDOUT:{}'.format(stdout))

            call(["pkill", "rosbag"])
            call(["pkill", "record"])
            self.gzstats.terminate()

    def killroscore(self):
        """ 
        layout.killroscore()

        Class method `killroscore` terminates roscore
        """

        if self.callflag["roscore"]:
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

    def physicsengine(self):
        '''
        layout.physicsengine()

        Class method `physicsengine` starts the physics engine simulator.
        Currently, only Gazebo is supported. The plan will be to add Unreal Engine and Unity in the future.

        Sets the `layout.callflag["physicsengine"]` to `True`.
        '''
         #Object to launch empty world
        emptylaunch = launch(launchfile = self.package_path + "/launch/empty.launch")

        emptylaunch.start()

        print('Empty world launched.')
        # The log call to true once log is called
        self.callflag["physicsengine"] = True
        time.sleep(5)

    def rviz(self, config = "/home/ivory/VersionControl/catvehicle_ws/src/sparkle/config/magna.rviz"):
        '''
        layout.rviz()

        Class method `rviz` starts the ros-rviz for visualizing vehicle's path, point cloud, etc.

        Sets the `layout.callflag["rviz"]` to `True`.

        Parameters
        ---------------
        cofig: `string`
            filepath of the rviz configuration file.

        Example
        ---------------
        >>> L = layout()
        >>> L.rviz(config="/home/ivory/catvehicle_ws/src/catvehicle/config/catvehicle.rviz")

        '''
        print("Start ros-rviz")
        self.rviz_process = subprocess.Popen(["sleep 3; rosrun rviz rviz  -d " +config], stdout=subprocess.PIPE, shell=True)
        self.rviz_pid = self.rviz_process.pid
        self.callflag["rviz"] = True

    def create(self, uri="localhost", port=11311):
        '''
        layout.create(uri="localhost", port=11311)
        
        Class method `create()` creates the simulation environment. If roscore has not started, then this function starts roscore as well by calling `layout.roscore()`.  If there is an already running roscore for given uri and port, it doesn't start the roscore.

        It also checks if a physics world (e.g. Gazebo world) is present or not, if not, it also calls `layout.phsyicsengine()` to create a simulated physics world. 
        
        Finally `create` sets desired physics   properties such as max step size and max update rate.
        
        Parameters
        -------------
        uri: `string`
            Specifies ROS_MASTER_URI
            A valid IP address string or resolvable hostname
        port: `integer` [0-65535]
            port number where roscore will start
            A valid port ranges from 0 to 65535 but note that not all port numbers are allowed.

        Example
        ----------

        >>> L = layout()
        >>> L.create("ivory.local", 11321 )

        >>> M = layout()
        >>> remoteURI = "150.135.222.42"
        >>> port = 134
        >>> M.create(remoteURI, port)

        See Also
        -----------
        roscore: starts/connect to the roscore on given URI and port.
        physicsengine: starts a simulated physics world (e.g. Gazebo world)

        '''
        # If roscore has not started yet, then start the roscore
        if not self.callflag["roscore"]:
            self.roscore(uri, port)

        time.sleep(3)

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Create a new ROS Node
        rospy.init_node('sparkle_layout', anonymous=True)
        roslaunch.configure_logging(self.uuid)

        if not self.callflag["physicsengine"]:
            time.sleep(3)
            self.physicsengine()

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
            print("Max Update Rate was not set properly, terminating simulation. Please restart the simulation.")
            sys.exit()

    def spawn(self):
        '''
        `layout.spawn()` spawns the simulated vehicles in the physics world.
        '''
        # #Object to spawn catvehicle in the empty world
        self.launch_obj = []
        for n in range(0, self.n_vehicles):
            launch_itr = launch(launchfile=self.package_path + '/launch/sparkle_spawn.launch', \
                X  = self.X[n], Y=self.Y[n], yaw = self.Yaw[n], robot = self.name[n], \
                    laser_sensor = False, updateRate = self.update_rate)
            self.launch_obj.append(launch_itr)
        
        for n in range(0, self.n_vehicles):
            self.launch_obj[n].start()
            call(["rosparam", "set", "/director/"+self.name[n], "false"])

    def control(self, **kwargs):
        '''
        Class methods specifies control algorithm for imparting velocity to the car.
        The Control Algorithm can be either uniform, OVFTL (Optimal-Velocity Follow-The-Leader) Model, FollowerStopper or anything else.

        kwargs:
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

        control_method: "uniform" | "ovftl" | "followerstopper"
            specifies **vehicle following algorithm** as control method.

            *"uniform"*: All vehicles in the circuit will have same velocity and steering angle

            *"ovftl"*: Specifies Optimal-Velocity Follow-The-Leader model.

            *"followerstopper"*: Followerstopper algorithm, not implemented yet.
        '''
        leader_vel = kwargs.get("leader_vel", 3.0)
        str_angle = kwargs.get("str_angle", 0.0)
        control_method = kwargs.get("control_method", "uniform")
        logdata = kwargs.get("logdata", False)


        if logdata:
            logdir = kwargs.get("logdir", "./")

            # We will start ROSBag record immediately
            self.log(logdir=logdir, prefix=self.package_name)

        self.launchvel_obj = [] # An array of launch object for starting and shutting down roslaunchs as required

        if control_method == "uniform":
            follower_vel = leader_vel
            n = 0
            launchvel_itr_0 = launch(launchfile= self.package_path + '/launch/stepvel.launch', \
                constVel = 0.0, strAng = str_angle, robot = self.name[n])

            self.launchvel_obj.append(launchvel_itr_0)
            self.launchvel_obj[0].start()
            print('Velocity node ' + str(0) + '  started.')

            for n in range(1, self.n_vehicles):
                launchvel_itr = launch(launchfile= self.package_path + '/launch/stepvel.launch', \
                constVel = 0.0, strAng = str_angle, robot = self.name[n])
                self.launchvel_obj.append(launchvel_itr)

            for n in range(1, self.n_vehicles):
                print('Velocity node ' + str(n) + '  started.')
                self.launchvel_obj[n].start()
            
            for n in range(0, self.n_vehicles):
                rosparamset = subprocess.Popen(["rosparam set /" +self.name[n]+"/constVel " + str(leader_vel)  ],   stdout=subprocess.PIPE, shell=True)

            self.callflag["startVel"] = True

        elif control_method == "ovftl":
            initial_distance = kwargs["initial_distance"]
            n= 0

            if self.__class__.__name__ == "circle":
                print('self.name[self.n_vehicles - 1] : {}'.format(self.name[self.n_vehicles - 1]))
                launchvel_itr_0 = launch(launchfile=self.package_path+'/launch/carfollowing.launch', this_name = self.name[n], \
                    leader_name = self.name[self.n_vehicles - 1], initial_distance =initial_distance, steering = str_angle, \
                         leader_x_init =self.X[self.n_vehicles-1], leader_y_init = self.Y[self.n_vehicles-1],  this_x_init =self.X[n], \
                              this_y_init =self.Y[n],  leader_odom_topic  = '/' + self.name[self.n_vehicles - 1] + '/setvel', \
                                  this_odom_topic = '/' + self.name[n] + '/setvel')
                self.launchvel_obj.append(launchvel_itr_0)

            elif self.__class__.__name__ == "lane":

                if not "leader_vel" in kwargs:
                    launchvel_itr_0 = launch(launchfile=self.package_path+'/launch/bagplay.launch', robot = self.name[n])
                else:
                    launchvel_itr_0 = launch(launchfile= self.package_path + '/launch/stepvel.launch', \
                constVel = 0.0, strAng = 0.0, robot = self.name[0])

                self.launchvel_obj.append(launchvel_itr_0)

            for n in range(1, self.n_vehicles):
                launchvel_itr = launch(launchfile=self.package_path+'/launch/carfollowing.launch', this_name = self.name[n], \
                    leader_name = self.name[n - 1], initial_distance =initial_distance, steering = str_angle, \
                         leader_x_init =self.X[n-1], leader_y_init = self.Y[n-1],  this_x_init =self.X[n], \
                              this_y_init =self.Y[n],  leader_odom_topic  = '/' + self.name[n - 1] + '/setvel', \
                                  this_odom_topic = '/' + self.name[n] + '/setvel')
                self.launchvel_obj.append(launchvel_itr)

            for n in range(1, self.n_vehicles):
                print('Velocity node ' + str(n) + '  started.')
                self.launchvel_obj[n].start()
            self.launchvel_obj[0].start()

            self.callflag["startVel"] = True
            rosparamset = subprocess.Popen(["rosparam set /" +self.name[0]+"/constVel " + str(leader_vel)  ],   stdout=subprocess.PIPE, shell=True)
            call(["rosparam", "set", "/execute", "true"])

        elif control_method == "injector":
            print("Injection control begin ...")
            injection_files = kwargs.get("injection_files")
            if len(injection_files) < self.n_vehicles:
                raise ValueError("Number of speed profile provided to inject speed to each vehicle is less than the number vehicles")

            time_col = kwargs.get("time_col")
            vel_col = kwargs.get("vel_col")
            rospack = rospkg.RosPack()
            package_path = rospack.get_path("sparkle")
            for n in range(0, self.n_vehicles): 
                launchvel_itr= launch(launchfile=package_path + '/launch/velinjector.launch', \
                    csvfile = injection_files[n], time_col = time_col, vel_col = vel_col, robot = self.name[n], str_angle = str_angle, decoupled = True)

                self.launchvel_obj.append(launchvel_itr)
            
            for n in range(0, self.n_vehicles):
                self.launchvel_obj[n].start()
                rospy.loginfo('Velocity node ' + str(n) + '  started.')
                print('Velocity node ' + str(n) + '  started.')

            self.callflag["startVel"] = True
            call(["rosparam", "set", "/execute", "true"])

    def destroy(self, sig=signal.SIGINT):
        '''
        Class method `destroy` terminates the simulation in the stack order of the various states of simulation were created.

        Parameters
        --------------
        sig: `signal`
            Pressing CTRL-C or SIGINT invokes destroy function.
        '''
        time.sleep(5)
        print('SIGINT: Destroying the physics world and terminating the simulation.')
        print('Terminating spawn launches')
        for n in range(0, self.n_vehicles):
            if self.callflag["startVel"]:
                self.launchvel_obj[n].shutdown()
                rospy.loginfo("Control for vehicle %s terminated", self.name[n])
            self.launch_obj[n].shutdown()

        if self.callflag["rviz"]:
            print("Destroying ros-rviz")
            self.rviz_process.terminate()
            call(["pkill", "rviz"])

        print('Destroying physics world')
        #kill the roscore
        self.gzclient.terminate()
        #Wait to prevent the creation of zombie processes.
        self.gzclient.wait()
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])

        self.stoplog()
        self.latesbag()
        self.killroscore()

        print("##### Simulation Terminated #####")

    def latesbag(self):
        '''
        Class method `latestbag` returns the latest bag file that saved ROS messages

        Returns
        ----------
        string:
            Path of the bag file.
        '''
        print("Retrieving latest bag file")
        if self.callflag["log"]:
            list_of_files1 = glob.glob('*.bag')
            list_of_files2 = glob.glob('*.bag.active')
            list_of_files = list_of_files1 + list_of_files2
            
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
                print("Renaming GZStat log file [{}] to retain bag file information".format(self.gzstatsfile))
                move_cmd = subprocess.Popen(["mv -v  " + self.gzstatsfile + " "+  fileName+"/" + fileName + "_gzStats.txt"],   stdout=subprocess.PIPE, shell=True)
                stdout = move_cmd.communicate()
                print('mv STDOUT:{}'.format(stdout))
                self.gzstatsfile = fileName+"/" +fileName + "_gzstats.txt"
                
                return self.logdir + "/" + latest_file
            else:
                print("No compatible bag file was found.")
                return None
        else:
            print("No bag was recorded in the immediate run.")
            return None

    def analyze(self):
        """
        Analyze and produce some insights into the simulation based on the log files (bagfiles, gazebo statistics, etc) recorded during this simulation
        """
        if self.gzstatsfile is None:
            print("No GZstats found. Check log directory.")
        else:
            print("GZStat file is {}".format(self.gzstatsfile))
            GZ = gzstats(self.gzstatsfile)
            GZ.plotRTF()
            GZ.plotSimStatus()

        if self.bagfile is None:
            print("No bag file was recorded in this session.")
        else:
            br = bagreader(self.bagfile)
            br.plot_vel(save_fig = True)
            br.plot_std(save_fig = True)
            br.plot_odometry(save_fig = True)
        

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

    simConfig = {"circumference": 450.0, "num_vehicle":  1, "update_rate": 25, "log_time": 120, "max_update_rate": 25, "time_step": 0.01, "laser": False, "description": "catlauch test run"}

    cl = layout(1, [10], [20], [0.023], **simConfig)
    print(cl.X)
    cl.create()
    cl.spawn()

    time.sleep(cl.log_time)

    print('Time to terminate')
    cl.destroy(signal.SIGINT)

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
