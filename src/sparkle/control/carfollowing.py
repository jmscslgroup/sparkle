#!/usr/bin/env python
# Initial Date: December 2019
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import rospy
import sys, getopt
import numpy as np
import glob
import os
from geometry_msgs.msg import Twist, Pose, Point, Vector3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class carfollowing(object):
    """
    CAR following constructor

    Parameters
    ----------
    thisvehicle: `string`,  Name of the ego vehicle

    leadervehicle: `string`,  Name of the leader vehicle

    Returns
    --------
    Nothing
    """

    def __init__(self, thisvehicle, leadervehicle, **kwargs):
        
        self.counter = 1.0 # for print log message at the time of publishing, currently we publish every 1000 steps or so.
        self.thisvehicle = thisvehicle
        self.leadervehicle= leadervehicle
        self.carfollowingmodel = kwargs.get("carfollowingmodel", None)
        rospy.init_node("car_following", anonymous=True)
        print("carfollowing node initialized with following vehicle [{}] and leader vehicle [{}]".format(thisvehicle, leadervehicle))

        self.FirstTime =  rospy.Time.now() # Capture time when this node is initialized.
        print("Time just after initialization is {}".format(self.FirstTime))

        self.init = False
        self.firstMessage =  {"leadervel": False, "vel":  False, "distance": False, "leaderaccel": False, "accel": False, "leaderodom": False, "odom": False}

        # Get all rosparams
        self.useSensorDistance = rospy.get_param('useSensorDistance', False)
        self.leaderposX = rospy.get_param('leaderX_init', 0.0)
        self.leaderposY = rospy.get_param('leaderY_init', 0.0)
        self.posX = rospy.get_param('X_init', 10.0)
        self.posY = rospy.get_param('Y_init', 10.0)     
        self.leaderaccel = rospy.get_param('leaderaccel_init',0.0) #initiliaze leader's acceleration
        self.accel = rospy.get_param('accel_init',0.0) #  # initialize the vehicle's acceleration
        self.str_angle = rospy.get_param('str_angle', 0.0) #steering angle
        
        #print(self.leaderposY)
        #print(self.posY)

        if self.useSensorDistance:
            self.distance = rospy.get_param('distance_init', 9.45) #initialize the distance between the vehicle and its leader
        else:
            self.distance = np.sqrt( (self.leaderposX - self.posX)**2 + (self.leaderposY - self.posY)**2   )

        #print("self.distance: {}".format(self.distance))

        V_init = self.V(self.distance)
        self.leadervel = 0.0 #V_init#initialize leader's velocity 
        self.vel = 0.0 #V_init #initialize velocity of this vehicle

        # Initialize Velocity Message
        self.vel_msg = Twist()
        self.vel_msg.linear.x = self.leadervel
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = self.str_angle

        # Initialize Acceleration Message
        self.accel_msg = Float64()
        self.accel_msg.data = 0.0
        
        # Initialize local variables
        leadervel = Twist()
        vel =Twist()
        distance = Float64()
        leaderodom = Odometry()
        odom = Odometry()
        leaderaccel = Vector3()
        accel = Vector3()

        odom.pose.pose.position.x =  self.posX
        odom.pose.pose.position.y =  self.posY

        leaderodom.pose.pose.position.x =  self.leaderposX
        leaderodom.pose.pose.position.y =  self.leaderposY

        leadervel.linear.x = self.leadervel
        vel.linear.x = self.vel

        distance.data = self.distance

        leaderaccel.x = self.leaderaccel
        accel.x = self.accel

        self.agentdata =  {"leadervel":  leadervel , "vel": vel, 
        "distance": distance, "leaderaccel":  leaderaccel, 
        "accel":  accel, "odom":  odom , 
        "leaderodom": leaderodom}

        # Create Publisher
        self.pubvel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pubaccel = rospy.Publisher('cmd_accel', Float64, queue_size=1)

        ## Create all subscribers
        config = ("leadervel", self.carfollowingmodel)
        rospy.Subscriber('leadervel', Twist, self.callback, config)

        config = ("vel", self.carfollowingmodel)
        rospy.Subscriber('vel', Twist, self.callback, config)

        config = ("leaderodom", self.carfollowingmodel)
        rospy.Subscriber('leaderodom', Odometry, self.callback, config)

        config = ("odom", self.carfollowingmodel)
        rospy.Subscriber('odom', Odometry, self.callback, config)
        
        config = ("distance", self.carfollowingmodel)
        rospy.Subscriber('distance', Float64, self.callback, config)

        config = ("leaderaccel", self.carfollowingmodel)
        rospy.Subscriber('leaderaccel', Point, self.callback, config)

        config = ("accel", self.carfollowingmodel)
        rospy.Subscriber('accel',Point, self.callback, config)
        
        # We also want to publish immediately after receiver a new data point
        self.publishNow = False

    def callback(self, data, args):
        """
        Callback function to receive data from its subscriber and parse it and store in the class's dictionary variable

        Parameters
        ------------
        data: `obj`, data sent by the susbscriber - type is determined during runtime

        type: `string`, expected type in string literal

        agent: `string`, expected agent who will be sending the data

        """

        agent = args[0]
        carfollowingmodel = args[1]
        #print("carfollowingmodel: {}".format(carfollowingmodel))
        
        if 'oldTime' not in self.__dict__:
            self.oldTime =  rospy.Time.now()
            return

        newTime = rospy.Time.now()
        deltaT = newTime - self.oldTime
        deltaT = deltaT.to_sec()
        #print("callback deltaT: {}".format(deltaT))
        net_deltaT = newTime - self.FirstTime
        net_deltaT = net_deltaT.to_sec()
        self.oldTime = newTime
     
        self.agentdata[agent] = data
        
        #print("\n-----------")
        #print("Agent is {}".format(agent))
        #print("Data: {}".format(data))
        #print("Args: {}".format(args))

        if self.useSensorDistance:
            self.distance = self.agentdata["distance"]
        else:
             self.distance = np.sqrt( (self.agentdata["leaderodom"].pose.pose.position.x - self.agentdata["odom"].pose.pose.position.x)**2 + (self.agentdata["leaderodom"].pose.pose.position.y - self.agentdata["odom"].pose.pose.position.y)**2   )
             self.agentdata['distance'].data = self.distance

        self.firstMessage[agent] = True

        # if all true in first message dictionary, then set self init to true
        if all(x is True for x in self.firstMessage.values()) and not self.init:
            #print("self.init: {}".format(self.init))
            #print('deltaT: {}'.format(deltaT))
            self.init = True
            return

        if carfollowingmodel is None:
            newvelocity, newaccel = self.cf_algorithm(self.L2Norm(self.agentdata['leadervel'].linear),    self.L2Norm(self.agentdata['leaderaccel']),
            self.L2Norm(self.agentdata['vel'].linear),   self.L2Norm(self.agentdata['accel']),
            self.agentdata['distance'].data, deltaT)
        else:
                newvelocity, newaccel = carfollowingmodel(self.L2Norm(self.agentdata['leadervel'].linear), 
                self.L2Norm(self.agentdata['leaderaccel']),
                self.L2Norm(self.agentdata['vel'].linear),
                self.L2Norm(self.agentdata['accel']),
                self.agentdata['distance'].data, deltaT)
    
        self.vel_msg.linear.x = newvelocity
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = self.str_angle

        self.accel_msg.data = newaccel

         # We always publish right away
        self.publishNow =True
       
    @staticmethod
    def L2Norm(vector3data):
        """
        Calculates the L2Norm of geometry_msgs Vector3

        Parameters
        -------------
        vector3data: `geometry_msgs/Vector3`, 3D Vector data whose L2 Norm is to be calculated

        Returns
        -----------
        L2 norm of the 3D vector data
        """
        magnitude = np.sqrt(vector3data.x**2 + vector3data.y**2 + vector3data.z**2)
        return magnitude

    def publish(self):
        if self.publishNow:
            self.counter = self.counter + 1
            if self.counter % 100== 0:
                #print("PublishNow:{}".format(self.publishNow))
                rospy.loginfo(rospy.get_caller_id() + " publishing new velocity {0} m/s.".format(self.vel_msg.linear.x))
            self.pubvel.publish(self.vel_msg)
            self.pubaccel.publish(self.accel_msg)
            # after we publish, we ensure to wait until a new odom point arrives
            self.publishNow = False

    @staticmethod
    def follower_stopper(leadervel, leaderaccel, vel, accel, distance, deltaT, **kwargs):
        """
        Implements follower stopper model

        Parameters
        -------------
        leadervel: `double` , leader's velocity at current time step
        
        leaderaccel: `double`, leader's acceleration at current time step

        vel: `double`, current vehicle's velocity at the current time step

        accel: `double`, current vehicle's acceleration at the current time step

        distance: `double`, distance bwteen leader vehicle and current vehicle at the current time step

        deltaT: `double`, timestep

        kwargs:  `dictionary`, variable argument dictionary

        Returns
        --------
        Car following model returns new commanded velocity for the next time step as per followerstopper
        """

        dx1 = 4.5
        dx3 = 6.0
        dx2 = (dx1 + dx3)/2.0

        a1 = 1.5
        a3 = 0.5
        a2 = (a1 + a3)/2.0

        cmd_vel = 0.0

        r = 6.5 # m/s

        v = np.min(np.max(leadervel, 0), r)

        if distance <= dx1:
            cmd_vel = 0.0
        elif (distance <=dx2) and (distance >=dx1):
            cmd_vel = v*((distance - dx1)/(dx2 - dx1))
        elif(distance <=dx3) and (distance >=dx2):
            cmd_vel = v + (r-v)*((distance - dx2)/(dx3 - dx2))

        elif distance > dx3:
            cmd_vel = r

    def cf_algorithm(self, leadervel, leaderaccel, vel, accel, distance, deltaT, **kwargs):
        """
        Implements a car following model

        Parameters
        -------------
        leadervel: `double` , leader's velocity at current time step
        
        leaderaccel: `double`, leader's acceleration at current time step

        vel: `double`, current vehicle's velocity at the current time step

        accel: `double`, current vehicle's acceleration at the current time step

        distance: `double`, distance bwteen leader vehicle and current vehicle at the current time step

        deltaT: `double`, timestep

        kwargs:  `dictionary`, variable argument dictionary

        Returns
        --------
        Car following model returns new commanded velocity for the next time step
        """ 

        b = 5.0 # [m^2/s] ,follow-the-leader strength
        a = 0.5  #  [1/s],  optimal velocity strength
        nu = 2.0 #power of distance in denominator of follow-the-leader term

        #print("cf_algorithm leadervel: {}".format(leadervel))
        #print("cf_algorithm vel: {}".format(vel))
        #print("cf_algorithm distance: {}".format(distance))

        accel = (b*(leadervel - vel)/(distance**nu)) + a*(self.V(distance) - vel)
        #print("cf_algorithm new acceleration: {}".format(accel))
        # print("cf_algorithm deltaT: {}".format(deltaT))
        #deltaT = 0.01
        deltav = accel*deltaT
        newvelocity = vel + deltav           
        #print("cf_algorithm New Velocity : {}".format(newvelocity))

        return newvelocity, accel

    @staticmethod
    def V(distance):
        """
        Optimal Velocity Function

        Parameters
        -------------
        distance: `double`, current distance between the current vehicle and the leader vehicle.

        Returns
        ----------
        Function returns the optimal Velocity
        """
        #print("V Function distance", distance)
        d0 =  5.00 # [m] reference vehicle distance
        Vm = 15/3.6 # [m/s] maximum velocity for optimal velocity functios
        V = Vm * (np.tanh((distance/d0) - 2) + np.tanh(2))/(1 + np.tanh(2))
        return V

def usage():
    print("carfollowing -t nebula -l magna -r 50.0")

def main(argv):
    #print("Main of carfollowing")
    thisvehicle='nebula'
    leadervehicle='magna'
    rate=20.0

    print('Argv: {}'.format(argv))
    try:
        opts, args = getopt.getopt(argv, "l:r:t:", ["leadervehicle", "rate", "thisvehicle"])
    except getopt.GetoptError:
        usage()
        exit.sys()

    for opt, arg in opts:
        if opt in ("-t", "--thisvehicle"):
            thisvehicle=arg
        elif opt in  ("-l", "--leadervehicle"):
            leadervehicle=arg
            print('arg={}'.format(arg))
        elif opt in ("-r", "-rate"):
            rate=arg
        else:
            usage()
            exit.sys()

    print("This Vehicle is {}.\nLeader Vehicle is {}\n".format(thisvehicle, leadervehicle))
    node = carfollowing(thisvehicle, leadervehicle)
    rosrate = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if rospy.get_param("/execute", False):

            #print("Sending velocity {} m/s to {}".format(node.vel_msg.linear.x, node.thisvehicle))
            node.publish()
            rosrate.sleep()
            
if __name__ == '__main__':
    main(sys.argv[1:])