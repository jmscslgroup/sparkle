#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

from circle import circle
from Bagplot import Bagplot
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
NUM_VEHICLES  = [5, 20]
UPDATE_RATE = [1]

# A set of six simulation with two cars and update rate varying from 20 to 120 Hz
for nn in NUM_VEHICLES: 
    for i in UPDATE_RATE:
        print("===================================")
        print("========   Simulation  Begin   ========")
        print("===================================")

        ## Simulation 1
        # Define Simulation Configuration
        simConfig = {"circumference": 260.0, "num_vehicle":  nn, "update_rate": i, "log_time": 90.0}
        cl = circle(simConfig["circumference"], simConfig["num_vehicle"])

        #Print the X coordinates of all vehicles for sanity checking
        print("X coordinates of vehicles spawned: ", cl.X)

        #Print the Y coordinates of all vehicles for sanity checking
        print("Y coordinates of vehicles spawned: ", cl.Y)

        # Start a simulation
        bagFile = cl.startSim1(simConfig["update_rate"], simConfig["log_time"] )

        if bagFile is not None:
            Bag  = Bagplot(bagFile)
            datafiles = Bag.getDataFile(fileFilter="magna-setvel")
            Files.append(datafiles)
            Bag.plotData(datafiles, 'PoseY', fileFilter='magna-setvel')

        configFileToSave = cl.bagfile[0:-4] + "/" + "simConfig.yaml"

        with open(configFileToSave, 'w') as file:
            documents = yaml.dump(simConfig, file)

        time.sleep(7)

# Now since we have all the bag files

Files = reduce(add, Files)

Bag.plotData(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')


