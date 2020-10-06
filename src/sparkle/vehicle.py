#!/usr/bin/env python
# Initial Date: March 4, 2020
# Author: Rahul Bhadani
# Copyright (c)  Rahul Bhadani, Arizona Board of Regents
# All rights reserved.


import roslaunch
import rospy, rosbag
import rospkg
import rostopic

import numpy
import glob
import datetime
import os
import sys, math, time
import signal
import subprocess, shlex
from subprocess import call

class vehicle:
    '''
    `vehicle` class specifies the vehicle and other associated parameters of the vehicle.

    Parameters
    -------------

    name: `string`

    '''
    def __init__(name):
        pass
