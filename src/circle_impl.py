#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from Bagplot import plot_timeseries

import signal
import sys, math, time
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import yaml
from operator import add

from api import sparkle_sim

Files = []

print("======   Simulation  Begin   =======")

simulation_description = "Verifying repeatablity if simulation in absence and presence of the laser with varying number of vehicles and different configuration"

# datafiles = sparkle_sim(num_vehicles=1, publish_rate= 1.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=False, description=simulation_description)
# Files.append(datafiles)

# datafiles = sparkle_sim(num_vehicles=1, publish_rate= 1.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=True, description=simulation_description)
#Files.append(datafiles)

# datafiles = sparkle_sim(num_vehicles=10, publish_rate= 1.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=False, description=simulation_description)
#Files.append(datafiles)

# datafiles = sparkle_sim(num_vehicles=10, publish_rate= 1.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=True, description=simulation_description)
#Files.append(datafiles)

datafiles = sparkle_sim(num_vehicles=10, publish_rate= 10.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=False,description=simulation_description)
Files.append(datafiles)

datafiles = sparkle_sim(num_vehicles=10, publish_rate= 10.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=True, description=simulation_description)
Files.append(datafiles)

# datafiles = sparkle_sim(num_vehicles=25, publish_rate= 10.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=False, description=simulation_description)
#Files.append(datafiles)

# datafiles = sparkle_sim(num_vehicles=25, publish_rate= 10.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=True, description=simulation_description)
#Files.append(datafiles)


# datafiles = sparkle_sim(num_vehicles=25, publish_rate= 50.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=False, description=simulation_description)
#Files.append(datafiles)


# datafiles = sparkle_sim(num_vehicles=25, publish_rate= 50.0, max_update_rate=25, time_step=0.01,  log_time= 120.0, include_laser=True,description=simulation_description)
#Files.append(datafiles)


# Now since we have all the bag files

Files = reduce(add, Files)

plot_timeseries(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')

print("Bag files captured in this run are:")
print(Files)
