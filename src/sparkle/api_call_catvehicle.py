#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from api import animate_catvehicle
from Bagplot import plot_ts
from operator import add
import matplotlib.pyplot as plt
plt.style.use('ggplot')
plt.rcParams["font.family"] = "Times New Roman"
import pandas as pd
import uuid

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

vehicle = 10
for i in range(1):
    datafile = animate_catvehicle( package_name="catvehicle", circumference = 230, n_vehicles=vehicle, leader_vel= 8.0, publish_rate=100.0, max_update_rate=100.0, time_step=0.01, log_time=300.0, include_laser=False, logdir = homedir + "/CyverseData/ProjectSparkle", description="CAT Vehicle Simulation: " + str(vehicle) + " Car(s), Iteration " + str(i) + " ID " + unique_runid)
    Files.append(datafile)
    print("Data received is: {}".format(datafile))

Files = reduce(add, Files)

plot_ts(Files, 'PoseY',Title=homedir+'/CyverseData/ProjectSparkle/Consolidated Plot', fileFilter='magna-odom')

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

