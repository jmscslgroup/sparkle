#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserv

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

def sparkle_sim(num_vehicles, publish_rate, max_update_rate, time_step, log_time, include_laser, description=""):
    """
    Sparkle Simulation API

     Parameters
    ----------
    num_vehicles: `integer`, The number vehicles to be spawned in the simulation

    publish_rate: `double`, The desired publish rate of the velocity of the vehicles' updated odometry to  update Gazebo simulation
    
    max_update_rate: `double`, Max Update Rate to be set in Gazebo Physics Simulation
    
    time_step: `double`, Time step for Gazebo Physics Simulation
    
    include_laser: `bool`, Boolean flag to include/exclude laser model and laser plugin on the first vehicle model
    
    description: `string`, Any human readable description of this simulation, the default is empty string, useful for annotating bag file name, will be saved  in the yaml file.
    
    Returns
    --------
    The name of the bag file is returned. We want all those bag files to create an aggregated plot of total simulation run.
    """

    print("Simulation beging")
   
    simConfig = {"circumference": 450.0, "num_vehicle":  num_vehicles, "update_rate": publish_rate, "log_time": log_time, "max_update_rate": max_update_rate, "time_step": 0.01, "include_laser": include_laser, "description": description}
    
    
    print("Simulation Configuration: {}".format(simConfig))
    
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
        Bag.plot_timeseries(datafiles, 'PoseY', fileFilter='magna-setvel')
        Bag.plot_topic_hz(datafiles)

    configFileToSave =  Circ.bagfile[0:-4] + "/" + "simConfig.yaml"

    with open(configFileToSave, 'w') as file:
        documents = yaml.dump(simConfig, file)

    time.sleep(6)
    pt.close('all')

    print("Simulation Ends. Datafiles saved are {}".format(datafiles))

    return datafiles