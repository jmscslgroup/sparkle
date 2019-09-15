#!/usr/bin/env python

# Author: Rahul Bhadani, Jonathan Sprinkle
# Maintainer Email: rahulbhadani@email.arizona.edu
# Initial Date: 6th August 2019
# What: This is a python script that plots the real time curve of yaw of the model
# Purpose of this plot is a mere diagnostics to check whether the yaw of the model
# being applied is the same as one designated by the three DOF simulink model.

import rospy
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
import sys, getopt
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.animation as animation
import time
import sys
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import scipy.special as sp
from gazebo_msgs.srv import GetModelState

class odom2path:

    def __init__(self, ns):
        self.ns = ns
        rospy.init_node('yawplot', anonymous=True)

        # Set up the figure
        #self.fig, self.ax = plt.subplots()
        self.fig =plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        # Empty array for storing yaws from model odom that we publish in odom.cc
        self.odom_yaw = []
        self.odom_time = []
        self.odom_X = []
        self.odom_Y = []

        # Empty array for storing model yaw published by Simulink on topic {}/angles
        self.model_yaw = []
        self.model_time = []

        # These flags will be true whenever a call back happens and go false whenever a
        # plotting happens
        self.odom_fetch = False
        self.model_fetch = False

        # Whenever we receive the odom and angles topic, the callback method is called
        rospy.Subscriber('odom'.format(ns), Odometry, self.odom_callback)
        rospy.Subscriber('angles'.format(ns), Point, self.angles_callback )

        # Get the model yaw from gazebo service call
        self.gazebo_yaw = []
        self.gazebo_time = []



    def odom_callback(self, data):

        self.model_fetch = True
        # Get the sim time from rospy.Time
        T = rospy.Time.now()
        secs = T.to_sec()*(10e9) #Convert to nanoseconds
        nsecs = T.to_nsec()

        total_t = secs + nsecs
        self.odom_X.append(data.pose.pose.position.x)
        self.odom_Y.append(data.pose.pose.position.y)

        self.odom_time.append(total_t)
        [roll, pitch, yaw] = self.calc_yaw( float(data.pose.pose.orientation.x),
                            float(data.pose.pose.orientation.y),
                            float(data.pose.pose.orientation.z),
                            float(data.pose.pose.orientation.w))
        self.odom_yaw.append(yaw)


    def angles_callback(self, data):

        self.odom_fetch = True
        # Get the sim time from rospy.Time
        T = rospy.Time.now()
        secs = T.to_sec()*(10e9) #Convert to nanoseconds
        nsecs = T.to_nsec()

        total_t = secs + nsecs

        self.model_time.append(total_t)
        yaw = float(data.z)
        self.model_yaw.append(yaw)


    def doplot(self):

        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            coordinates = model_coordinates(self.ns[1:], 'world')

            [qX, qY, qZ, qW] = [coordinates.pose.orientation.x, coordinates.pose.orientation.y, coordinates.pose.orientation.z, coordinates.pose.orientation.w]

            [roll, pitch, yaw] = self.calc_yaw( float(qX), float(qY), float(qZ), float(qW))

            self.gazebo_yaw.append(yaw)
            T = rospy.Time.now()
            secs = T.to_sec()*(10e9) #Convert to nanoseconds
            nsecs = T.to_nsec()

            total_t = secs + nsecs
            self.gazebo_time.append(total_t)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed: {0}".format(e))


        if (self.odom_fetch is True) and (self.model_fetch is True):

            # Limits data to xx items
            Time = self.odom_time[-5000:]
            Yaw = self.odom_yaw[-5000:]
            X_odom = self.odom_X[-5000:]
            Y_odom = self.odom_Y[-5000:]

            Yaw_model = self.model_yaw[-500:]
            Time_model = self.model_time[-500:]

            Yaw_gazebo = self.gazebo_yaw[-500:]
            Time_gazebo = self.gazebo_time[-500:]

            #rospy.loginfo("Odom2path: Plotting next data ")
            #self.ax.clear()
            #self.ax.plot(Time, Yaw, linestyle='--', color='firebrick', linewidth=2)
            #self.ax.set_axisbelow(True)
            #self.ax.minorticks_on()
            #self.ax.grid(which='major', linestyle='-', linewidth='0.5', color='salmon')
            #self.ax.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
            #plt.title('Model Yaw')
            #plt.xlabel('Time')
            #plt.ylabel('Yaw')

            self.ax.clear()
            self.ax.plot(X_odom, Y_odom, linestyle='--', color='firebrick', linewidth=2)
            self.ax.set_axisbelow(True)
            self.ax.minorticks_on()
            self.ax.grid(which='major', linestyle='-', linewidth='0.5', color='salmon')
            self.ax.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
            plt.title('Model Yaw')
            plt.xlabel('X')
            plt.ylabel('Y')


            #self.ax.plot(Time_gazebo, Yaw_gazebo, linestyle='--', color='darkseagreen', linewidth=2)

            #self.ax.plot(Time_model, Yaw_model, linestyle='--', color='deeppink', linewidth=2)

            #self.ax.legend(['odom.cc Yaw', 'Gazebo Service Call Yaw', 'Simulink Model Yaw'])


            self.ax.plot()
            plt.draw()
            plt.pause(0.00001)
            self.odom_fetch = False
            self.model_fetch = False

    def calc_yaw(self, qx, qy, qz, qw):
        # roll (x-axis rotation)
        sinr_cosp = 2.0*((qw * qx) + (qy * qz))
        cosr_cosp = 1  - 2* ( (qx * qx) + (qy * qy))
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * ( (qw* qy) - (qz * qx))
        if (np.fabs(sinp) >= 1.0):
            pitch = (np.pi/2.0)*np.sign(sinp) # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        # yaw (z-axis rotation)
        siny_cosp = 2.0*((qw * qz) + (qx * qy))
        cosy_cosp = 1.0 - 2.0*((qy * qy) + (qz * qz))
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appens '/' at the end as well,
    ns = ns[0:-1]
    node = odom2path(ns)

    # Run at 10 Hz
    rate = rospy.Rate(10)

    rospy.loginfo("Yawplot: Namaespace acquired is " + ns)
    while not rospy.is_shutdown():
        node.doplot()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
