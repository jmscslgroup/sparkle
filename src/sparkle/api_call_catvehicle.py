#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from api import animate_catvehicle
from operator import add
import matplotlib.pyplot as plt
plt.style.use('ggplot')
plt.rcParams["font.family"] = "Times New Roman"
import pandas as pd

Files = []

for i in range(1):
    datafile = animate_catvehicle( package_name="catvehicle", circumference = 230, num_vehicles=1, publish_rate=100.0, max_update_rate=100.0, time_step=0.01, log_time=20.0, include_laser=True, logdir = "/home/ivory/CyverseData/ProjectSparkle", description="CAT Vehice Simulation: Single Car")
    Files.append(datafile)
    print("Data received is: {}".format(datafile))


# Odom dataframe 
fig, ax = plt.subplots(1,1)
fig.tight_layout(pad=5.0)
for data in datafile:
    odom_df = pd.read_csv(data)
    plt.plot(odom_df['PoseX'],  odom_df['PoseY'], linewidth = 1)

plt.title('Trajectory of CAT Vehicle During Simulation', fontsize= 24)
ax.set_xlabel('x',fontsize=24)
ax.set_ylabel('y', fontsize=24)
plt.show()

