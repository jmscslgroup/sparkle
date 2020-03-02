#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from circle import circle
from Bagplot import plot_timeseries
from Bagplot import Bagplot
from GZStats import GZStats
import signal
import matlab.engine
import pandas as pd
import sys, math, time
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import yaml
from operator import add


Files = []
i = 1
NUM_VEHICLES  = [1]
UPDATE_RATE = [25]

# A set of six simulation with two cars and update rate varying from 20 to 120 Hz
for nn in NUM_VEHICLES: 
    for i in UPDATE_RATE:
        print("===================================")
        print("========   Simulation  Begin   ========")
        print("===================================")

        ## Simulation 1
        # Define Simulation Configuration
        simConfig = {"circumference": 260.0, "num_vehicle":  nn, "update_rate": i, "log_time": 30.0, "max_update_rate": 100.0, "time_step": 0.01, "include_laser": True, "description": "Trial"}
        #cl = circle(simConfig["circumference"], simConfig["num_vehicle"])
        Circ = circle(**simConfig)
        #Print the X coordinates of all vehicles for sanity checking
        print("X coordinates of vehicles spawned: ",  Circ.X)

        #Print the Y coordinates of all vehicles for sanity checking
        print("Y coordinates of vehicles spawned: ",  Circ.Y)

        # Start a simulation
        bagFile = Circ.start_circle_sim()

        gz_stat_file = Circ.gzstatsFile

        GZ = GZStats(gz_stat_file)
        GZ.plotRTF()
        GZ.plotSimStatus()

        if bagFile is not None:
            Bag  = Bagplot(bagFile)
            datafiles = Bag.getDataFile(fileFilter="magna-setvel", msg_types = "odom")
            Files.append(datafiles)
            Bag.plot_timeseries(datafiles, 'PoseY', fileFilter='magna-setvel')
            Bag.plot_topic_hz(datafiles)

        configFileToSave =  Circ.bagfile[0:-4] + "/" + "simConfig.yaml"

        with open(configFileToSave, 'w') as file:
            documents = yaml.dump(simConfig, file)
    
        time.sleep(7)

# Now since we have all the bag files

Files = reduce(add, Files)

plot_timeseries(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')

