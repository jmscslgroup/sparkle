#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from circle import circle
import signal
import matlab.engine
import pandas as pd
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import time

# Circumference of circle 
circum_circle = 260.0

# Number of  Vehicles to Spawn
n_vehicles = 1
cl = circle(circum_circle, n_vehicles)

cl.startROS()

cl.log()

time.sleep(10)

cl.stopLog()

cl.killROS()