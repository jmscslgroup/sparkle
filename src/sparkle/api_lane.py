#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from api import animate_lane
from operator import add
from Bagplot import plot_ts
import uuid
import matplotlib.pyplot as plt
import pandas as pd

Files = []
unique_runid = str(uuid.uuid4())

import os.path
homedir = os.path.expanduser("~")

import os
directory = homedir + "/CyverseData/ProjectSparkle"
if not os.path.exists(directory):
    os.makedirs(directory)

directory = homedir + "/CyverseData/ProjectSparkle/Consolidated Plot"
if not os.path.exists(directory):
    os.makedirs(directory)

vehicle = 2
for i in range(1):
    datafile = animate_lane(vehicle_spacing= 7.0, n_vehicles=vehicle, leader_vel= 7.0, publish_rate=100.0, max_update_rate=100.0, time_step=0.01, log_time=15.0, include_laser=False, logdir = homedir +"/CyverseData/ProjectSparkle", description="Sparkle Simulation: " + str(vehicle) + " Car(s), Iteration " + str(i) + " ID " + unique_runid)
    Files.append(datafile)
    print("Data received is: {}".format(datafile))