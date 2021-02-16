#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from .api import animate_circle
from operator import add
from .bagplot import plot_ts
from .plot_util import multi_plot
import uuid
import matplotlib.pyplot as plt
plt.style.use('ggplot')
plt.rcParams["font.family"] = "Times New Roman"
import pandas as pd
import os
import os.path

Files = []
unique_runid = str(uuid.uuid4())
homedir = os.path.expanduser("~")
directory = homedir + "/CyverseData/ProjectSparkle"
if not os.path.exists(directory):
    os.makedirs(directory)
directory = homedir + "/CyverseData/ProjectSparkle/Consolidated Plot"
if not os.path.exists(directory):
    os.makedirs(directory)

vehicle = 15
for i in range(2):
    bagfile = animate_circle(circumference = 260, n_vehicles=vehicle, leader_vel= 8.0, publish_rate=20.0, max_update_rate=10.0, time_step=0.01, log_time=30.0, include_laser=False, logdir = homedir +"/CyverseData/ProjectSparkle", description="Sparkle Simulation: " + str(vehicle) + " Car(s), Iteration " + str(i) + " ID " + unique_runid)
    Files.append(bagfile)

Files = reduce(add, Files)
print("Bag files captured in this run are: {}".format(Files))

#plot_ts(Files, 'pose.y', Title=homedir+'/CyverseData/ProjectSparkle/Consolidated Plot', fileFilter='magna-setvel')
multi_plot(Files, "odometry", "/magna/setvel", "pose.y")



# # Odom dataframe 
# fig, ax = plt.subplots(1)
# plt.style.use('seaborn')
# plt.rcParams[ 'font.family'] = 'Roboto'
# plt.rcParams[ 'font.weight'] = 'bold'
# params = {'legend.fontsize': 7, 'legend.handlelength': 2, 'legend.loc': 'upper right'}
# plt.rcParams.update(params)
# plt.rcParams["figure.figsize"] = (8,8)
# for data in Files:
#     odom_df = pd.read_csv(data)
#     plt.plot(odom_df['pose.x'],  odom_df['pose.y'], linewidth = 1, label = data)

# plt.title('Trajectory of  Vehicles During Simulation', fontsize= 15)
# ax.set_xlabel('x',fontsize=14)
# ax.set_ylabel('y', fontsize=14)
# ax.set_aspect('equal', 'box')
# plt.tight_layout()
# plt.legend()
# plt.show()


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

