#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from api import sparkle_sim
from operator import add
from Bagplot import plot_ts

Files = []

datafile = sparkle_sim(circumference = 230, num_vehicles=22, publish_rate=20.0, max_update_rate=20.0, time_step=0.01, log_time=200.0, include_laser=True, description="API Testing")
Files.append(datafile)
print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=30, publish_rate=100, max_update_rate=50, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=30, publish_rate=1, max_update_rate=100, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=30, publish_rate=100, max_update_rate=100, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=2, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = sparkle_sim(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))datafile

Files = reduce(add, Files)

plot_ts(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')

print("Bag files captured in this run are:")
print(Files)
