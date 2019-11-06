#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from circle import catlaunch
import signal
import matlab.engine
import pandas as pd
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style


# Circumference of circle 
circum_circle = 260.0

# Number of  Vehicles to Spawn
n_vehicles = 1
cl = catlaunch(circum_circle, n_vehicles)

#Print the X coordinates of all vehicles for sanity checking
print("X coordinates of vehicles spawned: ", cl.X)

# Spawn all vehicles 
cl.spawn()

# Start Rosbag record
cl.log()

# Start Timer
cl.startTimer(100.0)

# Enable the system
cl.enableSystem()

signal.signal(signal.SIGINT, cl.signal_handler)
print("Press Ctrl+C")
signal.pause()

if cl.logcall:

    try:
        bagFileSaved = cl.bagfile

        eng = matlab.engine.start_matlab()
        eng_return = eng.addpath('/home/ivory/VersionControl/Jmscslgroup/ROSBagReader')
        B = eng.ROSBagReader(bagFileSaved)

        eng.workspace["B"] = B
        topics = eng.eval("B.availableTopics")

        print("Topics available in Bagfiles are: "  + str(topics))

        DataFile = eng.eval("B.extractOdometryData()")

        print(DataFile)

        CSVFile = DataFile[0]

        data_frame = pd.read_csv(CSVFile)

        X = data_frame['PoseX']
        Y = data_frame['PoseY']

        Time = data_frame['Time']

        pt.rcParams["figure.figsize"] = (12,8)
        params = {'legend.fontsize': 18,
            'legend.handlelength': 2}
        pt.rcParams.update(params)
        pt.rcParams["font.family"] = "Times New Roman"
        fig =pt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.set_axisbelow(True)
        ax.minorticks_on()
        ax.tick_params(axis="x", labelsize=18)
        ax.tick_params(axis="y", labelsize=18)
        pt.grid(True)
        ax.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
        ax.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
        ax.set_xlabel('Time', fontsize=18)
        ax.set_ylabel('X', fontsize=18)
        ax.set_title("X Coordinate Timeseries:",fontsize= 20)

        pt.plot(Time, X,  color='firebrick', linewidth=1, linestyle='-', marker ='.', markersize=2 )
        pt.show()
    except AttributeError:
        print("No bagfile information found.")