#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from circle import catlaunch
import signal

# Circumference of circle 
circum_circle = 260.0

# Number of  Vehicles to Spawn
n_vehicles = 2
cl = catlaunch(circum_circle, n_vehicles)

#Print the X coordinates of all vehicles for sanity checking
print("X coordinates of vehicles spawned: ", cl.X)

# Spawn all vehicles 
cl.spawn()

signal.signal(signal.SIGINT, cl.signal_handler)
print("Press Ctrl+C")
signal.pause()