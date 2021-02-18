#!/usr/bin/env python
# Initial Date: February 2021
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.
import os
import uuid
import time
import signal

from sparkle import circle


homedir = os.path.expanduser("~")

# Specify where to save data
datadir = homedir +"/CyverseData/ProjectSparkle/RTFvsNVehicles"

n_vehicles = 1

include_laser = False
circumference = 260
max_update_rate = 100.0 # for gazebo
time_step = 0.01 # for gazebo
leader_vel = 12.0
log_time = 60.0
update_rate = 20.0 # for throttling
unique_runid = str(uuid.uuid4())
description = "Circular Trajectory, Sparkle, Unique run ID = " + unique_runid

sim_config = {"n_vehicles":  n_vehicles, 
                  "include_laser": include_laser, 
                  "circumference": circumference,
                  "max_update_rate": max_update_rate,
                  "time_step": time_step, 
                  "leader_vel": leader_vel,
                  "log_time": log_time, 
                  "update_rate": update_rate,
                  "description": description,
                  "package_name": "sparkle",
                  "logdir": datadir}

# I will be running cars in a circle
C = circle(**sim_config)

# Create the simulation
C.create()

# Spawn the simulation
C.spawn()

# Control all the cars
C.control(leader_vel=leader_vel, logdir = datadir, str_angle = C.const_angle, logdata = False)

# Start visualization
C.rviz(config = C.package_path + "/config/magna.rviz")

# Run the simulation for`log_time`
time.sleep(log_time)

# Destroy the simulation
C.destroy(signal.SIGINT)
