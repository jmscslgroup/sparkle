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


print("===================================")
print("======   Simulation  Begin   =======")
print("===================================")

NUM_VEHICLES  = [5, 15, 20, 30]
UPDATE_RATE = [0.5, 1, 10, 25, 50, 100]
MAX_UPDATE_RATE = [25, 50, 100] # This is for Gazebo


NUM_VEHICLES  = [5]
UPDATE_RATE = [0.25]
MAX_UPDATE_RATE = [25] # This is for Gazebo



for num_vehicles in NUM_VEHICLES:
    for update_rate in  UPDATE_RATE:
        for max_update_rate in MAX_UPDATE_RATE:
            ## Simulation 1
            # Define Simulation Configuration
            simConfig = {"circumference": 450.0, "num_vehicle":  num_vehicles, "update_rate": update_rate, "log_time": 120.0, "max_update_rate": max_update_rate, "time_step": 0.01}
            #cl = circle(simConfig["circumference"], simConfig["num_vehicle"])
            Circ = circle(**simConfig)

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

            time.sleep(6)
            pt.close('all')

# Now since we have all the bag files

Files = reduce(add, Files)

plot_timeseries(Files, 'PoseY', Title='Consolidated Plot', fileFilter='magna-setvel')

print("Bag files captured in this run are:")
print(Files)


