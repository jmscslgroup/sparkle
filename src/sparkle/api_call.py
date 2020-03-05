#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from api import animate_circle
from operator import add
from Bagplot import plot_ts
import uuid
import matplotlib.pyplot as plt
plt.style.use('ggplot')
plt.rcParams["font.family"] = "Times New Roman"
import pandas as pd

Files = []
unique_runid = str(uuid.uuid4())
vehicle = 5
for i in range(5):
    datafile = animate_circle(circumference = 230, n_vehicles=vehicle, publish_rate=20.0, max_update_rate=100.0, time_step=0.01, log_time=300.0, include_laser=False, logdir = "/home/reu-cat/CyverseData/ProjectSparkle", description="CAT Vehicle Simulation:" + str(vehicle) + "Car, Iteration " + str(i) + " ID " + unique_runid)
    Files.append(datafile)
    print("Data received is: {}".format(datafile))

Files = reduce(add, Files)

plot_ts(Files, 'PoseY', Title='/home/reu-cat/CyverseData/ProjectSparkle/Consolidated Plot', fileFilter='magna-setvel')

print("Bag files captured in this run are:")
print(Files)


# Odom dataframe 
fig, ax = plt.subplots(1,1)
fig.tight_layout(pad=5.0)
for data in Files:
    odom_df = pd.read_csv(data)
    plt.plot(odom_df['PoseX'],  odom_df['PoseY'], linewidth = 1, label = data)

plt.title('Trajectory of CAT Vehicle During Simulation', fontsize= 24)
ax.set_xlabel('x',fontsize=24)
ax.set_ylabel('y', fontsize=24)
plt.legend()
plt.show()


# datafile = animate_circle(num_vehicles=30, publish_rate=100, max_update_rate=50, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=30, publish_rate=1, max_update_rate=100, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=30, publish_rate=100, max_update_rate=100, time_step=0.01, log_time=400, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=2, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=10, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=10, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=25, publish_rate=1, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=True, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))

# datafile = animate_circle(num_vehicles=25, publish_rate=25, max_update_rate=25, time_step=0.01, log_time=300, include_laser=False, description="API Testing")
# Files.append(datafile)
# print("Data received is: {}".format(datafile))datafile

