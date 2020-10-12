#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Header, Float64
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
import pandas as pd


#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import rospy
import sys, time
import numpy as np


class velinjector():
    def __init__(self, ns, csvfile, time_col, vel_col, str_angle):
        """
        `injector` injects velocity from a csv file to a vehicle in the simulation
        """
        self.ns = ns
        rospy.init_node("injector", anonymous=True)

        ## Publishers
        self.vel_pub = rospy.Publisher('cmd_vel'.format(ns), Twist, queue_size=1)
        self.str_angle = str_angle
        dataframe = pd.read_csv(csvfile)
        dataframe.dropna(inplace=True)
        time = None
        velocity = None
        if time_col in dataframe.columns:
            time = dataframe[time_col]
        else:
            raise KeyError("{} column not available in {}".format(time_col, csvfile))
        
        if vel_col in dataframe.columns:
            velocity = dataframe[vel_col]
        else:
            raise KeyError("{} column not available in {}".format(vel_col, csvfile))
        
        self.dataframe = dataframe
        self.time = time
        self.velocity = velocity
        self.speed = pd.DataFrame()
        self.speed['Time'] = time
        self.speed['Message'] = velocity

        ## Check for monotonicity of time
        time_diff = np.diff(self.time)
        if not np.all(time_diff) >= 0:
            raise ValueError("Time is not monotonically increasing in the provided dataset")

        self.current_time = None
        self.next_time = None
        
    def publish(self):
        """
        Publish Function 
        """

        new_vel = self.speed.iloc[0]['Message']
        self.current_time = self.speed.iloc[0]['Time']

        if self.speed.shape[0] == 1:
            self.next_time = -1 # -1 will signify that it is time to step publishing when last row has been read
            return

        self.next_time = self.speed.iloc[1]['Time']

        # Remove the row just read from the dataframe
        self.speed = self.speed.iloc[1:]

        new_vel_msg = Twist()
        new_vel_msg.linear.x = new_vel
        new_vel_msg.linear.y = 0.0
        new_vel_msg.linear.z = 0.0
        new_vel_msg.angular.x = 0.0
        new_vel_msg.angular.y = 0.0
        new_vel_msg.angular.z =  self.str_angle

        self.vel_pub.publish(new_vel_msg)

        

def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appends '/' at the end as well,
    ns = ns[0:-1]
  
    csvfile = argv[0]
    time_col = argv[1]
    vel_col = argv[2]
    str_angle = float(argv[3])
    node = velinjector(ns, csvfile, time_col, vel_col, str_angle)

    #rate = rospy.Rate(20) # 20 Hz publish rate
    
    while not rospy.is_shutdown():
        if rospy.get_param("/execute", False):
            node.publish()
            if node.next_time == -1:
                break
            deltaT = node.next_time - node.current_time
            time.sleep(deltaT)

if __name__ == '__main__':
    main(sys.argv[1:])
