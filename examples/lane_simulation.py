#!/usr/bin/env python
# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import os
import uuid
import time
import signal

from sparkle import api
from sparkle import gzstats
from sparkle import lane

homedir = os.path.expanduser("~")

# Specify where to save data
datadir = homedir +"/CyverseData/ProjectSparkle/RTFvsNVehicles"
n_vehicles = 4

include_laser = False
vehicle_spacing = 20.0
max_update_rate = 100.0 # for gazebo
time_step = 0.01 # for gazebo
leader_vel = 2.0
log_time = 55.0
update_rate = 20.0 # for throttling
unique_runid = str(uuid.uuid4())
description = "Lane Trajectory, Sparkle, Unique run ID = " + unique_runid

datadir = homedir +"/CyverseData/ProjectSparkle/RTFvsNVehicles"

sim_config = {"n_vehicles":  n_vehicles, 
                  "include_laser": include_laser, 
                  "vehicle_spacing": vehicle_spacing,
                  "max_update_rate": max_update_rate,
                  "time_step": time_step, 
                  "leader_vel": leader_vel,
                  "log_time": log_time, 
                  "update_rate": update_rate,
                  "description": description,
                  "package_name": "sparkle",
                  "logdir": datadir}

L = lane(**sim_config)



L.simulate(leader_vel, logdir = datadir, logdata = False)

