#!/usr/bin/env python

# Author: Rahul Bhadani, Jonathan Sprinkle
# Maintainer Email: rahulbhadani@email.arizona.edu
# Initial Date: 2nd August 2019
# What: This is a python script that publishes path information based on odometry data
# This node publishes at 20 Hz whenever the odometry differs by at least 1m in L1 norm.

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg  import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
import sys, getopt

class odom2path:

    def __init__(self, ns):
        self.ns = ns
        rospy.init_node('odom2path', anonymous=True)

        # Whenever we receive the odom topic, the callback method is called
        rospy.Subscriber('odom'.format(ns), Odometry, self.callback)

        # Set up the state data for the publisher
        self.pub_path = rospy.Publisher('path'.format(ns), Path, queue_size=10)

        # We also want to publish immediately after receiver a new data point
        self.publishNow = True

        # Initialize the path message and its header
        self.pathMsg = Path()
        self.pathMsg.header = Header()
        self.x = None
        self.y = None

    def callback(self, data):
        # We always publish right away
        self.publishNow =True

        # Increment the header sequence
        self.pathMsg.header.seq +=1

        # Get the sim time from rospy.Time
        self.pathMsg.header.stamp = rospy.Time.now()

        # Set the odometry frame
        self.pathMsg.header.frame_id = '{0}/odom'.format(self.ns)

        # We append new pose to the path only when the position has
        # moved more than 1m from its previous spot

        if self.x == None or (  (abs(self.x  - data.pose.pose.position.x) > 1) or (abs(self.y - data.pose.pose.position.y) > 1)):
            pose = PoseStamped()

            # copy over the values individually
            pose.header.frame_id = '{0}/odom'.format(self.ns)
            pose.header.seq = len(self.pathMsg.poses)+1
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = float(data.pose.pose.position.x)
            pose.pose.position.y = float(data.pose.pose.position.y)
            pose.pose.orientation.x = float(data.pose.pose.orientation.x)
            pose.pose.orientation.y = float(data.pose.pose.orientation.y)
            pose.pose.orientation.z = float(data.pose.pose.orientation.z)
            pose.pose.orientation.w = float(data.pose.pose.orientation.w)

            self.pathMsg.poses.append(pose)

            self.x = float(data.pose.pose.position.x)
            self.y = float(data.pose.pose.position.y)

    def publish(self):
        if self.publishNow:
            rospy.logdebug(rospy.get_caller_id() + " publishing new path with {0} elements.".format(len(self.pathMsg.poses)))
            self.pub_path.publish(self.pathMsg)

            # after we publish, we ensure to wait until a new odom point arrives
            self.publishNow = False

def main(argv):
    ns = rospy.get_namespace() #Retrieve namespace this way appens '/' at the end as well,
    ns = ns[0:-1]
    node = odom2path(ns)

    # Run at 10 Hz
    rate = rospy.Rate(10)

    rospy.loginfo("Odom2path: Namespace acquired is " + ns)
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
