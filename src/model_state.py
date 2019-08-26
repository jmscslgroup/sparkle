#!/usr/bin/env python
# Author: Rahul Bhadani
# Maintainer Email: rahulbhadani@email.arizona.edu
# Initial Date: 25th August 2019
# What is this: A python script for service call and setting gazebo model positions

import rospy
import numpy as np
from std_msgs.msg import String, Header
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Quaternion
from gazebo_msgs.msg import ModelState
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

class SetVehicleState:

    def __init__(self, ns):
        self.ns = ns
        rospy.init_node('yawplot', anonymous=True)

        self.newMessage = False

        self.Time = rospy.Time.now()
        self.pose = []
        self.orientation = []
        self.twist = []

        self.coordinates = []
        self.sub = rospy.Subscriber('setvel'.format(ns), Odometry, self.setVel_callback)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)

    def setVel_callback(self, data):
        self.newMessage = True
        self.Time = rospy.Time.now()

        self.pose = data.pose.pose.position;
        self.orientation = self.calcQuaternion(data.pose.pose.orientation.x,
                                data.pose.pose.orientation.y,
                                data.pose.pose.orientation.z)
        self.twist = data.twist.twist

    def calcQuaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        q = Quaternion(x, y, z, w)

        return q

    def setState(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.coordinates = model_coordinates(self.ns[1:], 'world')

            [qX, qY, qZ, qW] = [self.coordinates.pose.orientation.x, self.coordinates.pose.orientation.y, self.coordinates.pose.orientation.z, self.coordinates.pose.orientation.w]

            [pX, pY, pZ] = [self.coordinates.pose.position.x, self.coordinates.pose.position.y, self.coordinates.pose.position.z]

            if self.newMessage is True:
               gm = ModelState()
               gm.model_name = self.ns[1:]
               gm.reference_frame = 'world'
               gm.twist = self.twist


               '''
               gm.pose.position.x = pX
               gm.pose.position.y = pY
               gm.pose.position.z = pZ
               gm.pose.orientation.x = qX
               gm.pose.orientation.y = qY
               gm.pose.orientation.z = qZ
               gm.pose.orientation.w = qW
               '''

               gm.pose.position = self.pose
               gm.pose.orientation = self.orientation

               self.pub.publish(gm)
        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service failed: {0}".format(e))



def main(argv):
    ns = rospy.get_namespace() # Retrieve namespace this way appends '/' at the end as well
    ns = ns[0:-1]
    node = SetVehicleState(ns)

    rate = rospy.Rate(60)

    rospy.loginfo("model_state.py: Namespace acquired is " + ns)

    while not rospy.is_shutdown():
        node.setState()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])


