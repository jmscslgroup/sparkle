#!/usr/bin/env python
# 
# Author: Rahul Bhadani
# Copyright (c) Arizona Board of Regents
# All rights reserved.

import sys
import rospy
import csv
import time
import datetime
import socket
from os.path import expanduser

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties

class saveparam:
    def __init__(self, ns):
        """
        """
        pass
    
def main(argv):

    rospy.wait_for_service('gazebo/get_physics_properties')
    rospy.wait_for_service('gazebo/set_physics_properties')

    
    max_update_rate = float(argv[0])
    time_step = float(argv[1])

    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()

    time.sleep(4)

    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()
    print("Current  max_update_rate is {}".format( physics_properties.max_update_rate))

    while(physics_properties.max_update_rate != max_update_rate):
        print("Max Update Rate was not set properly, terminating simulation. Please restart the simulation.")
        physics_properties.max_update_rate = max_update_rate
        physics_properties.time_step = time_step

        set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics_properties_prox(physics_properties.time_step,
                                    physics_properties.max_update_rate,
                                    physics_properties.gravity,
                                    physics_properties.ode_config)

    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()
    print("New  max_update_rate is {}".format( physics_properties.max_update_rate))

if __name__ == '__main__':
    main(sys.argv[1:])
