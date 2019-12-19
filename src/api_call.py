#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from Bagplot import plot_timeseries
from api import sparkle_sim

Files = []

datafile = sparkle_sim(num_vehicles=2, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=2, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=False, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=False, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=360, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=360, include_laser=False, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=360, include_laser=False, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=360, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

datafile = sparkle_sim(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=360, include_laser=False, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

Files = reduce(add, Files)

plot_timeseries(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')

print("Bag files captured in this run are:")
print(Files)
