#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved

from circle import circle
from catvehicle import catvehicle
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

def animate_circle(circumference, num_vehicles, publish_rate, max_update_rate, time_step, log_time, include_laser, logdir, description=""):
    '''
    Sparkle Simulation API: `animate_circle`

    Higher level API for performing simulation on a circular trajectory

    Parameters
    --------------
    num_vehicles: `integer`
         The number vehicles to be spawned in the simulation

    publish_rate: `double`
        The desired publish rate of the velocity of the vehicles' updated odometry to  update Gazebo simulation
    
    max_update_rate: `double`
        Max Update Rate to be set in Gazebo Physics Simulation
    
    time_step: `double`
        Time step for Gazebo Physics Simulation
    
    include_laser: `bool`
        Boolean flag to include/exclude laser model and laser plugin on the first vehicle model

    logdir: `string`
        Directory/Path where bag files and other data files recording simulation statistics will be saved.
    
    description: `string`
        Any human readable description of this simulation, the default is empty string, useful for annotating bag file name, will be saved  in the yaml file.
    
    Returns
    ----------
    `string`
        A list of data files is returned. We can specify only certain information to retrieved from saved bag files at the end of the simulation.

   '''

    print("Simulation beging")
   
    simConfig = {"circumference": circumference, "num_vehicle":  num_vehicles, "update_rate": publish_rate, "log_time": log_time, "max_update_rate": max_update_rate, "time_step": time_step, "include_laser": include_laser, "logdir": logdir, "description": description}
    
    
    print("Simulation Configuration: {}".format(simConfig))
    
    C = circle(**simConfig)

    # Start a simulation
    bag = C.simulate(logdir=logdir)

    print("Bagfile recorded is {}".format(bag))

    gz_stat_file = C.gzstatsfile
    print("GZStat file is {}".format(gz_stat_file))
    GZ = GZStats(gz_stat_file)
    GZ.plotRTF()
    GZ.plotSimStatus()

    datafiles = []
    if bag is not None:
        Bag  = Bagplot(bag)
        datafiles = Bag.getDataFile(fileFilter="magna-setvel", msg_types = "odom")
        Bag.plot_timeseries(datafiles, 'PoseY', fileFilter='magna-setvel')
        Bag.plot_topic_hz(datafiles)

        configfile =  C.bagfile[0:-4] + "/" + "simConfig.yaml"

        with open(configfile, 'w') as file:
            documents = yaml.dump(simConfig, file)

    time.sleep(6)
    pt.close('all')

    print("Simulation Ends. Datafiles saved are {}".format(datafiles))

    return datafiles

def animate_catvehicle(package_name, circumference, num_vehicles, publish_rate, max_update_rate, time_step, log_time, include_laser, logdir, description=""):
    '''
    Sparkle Simulation API: `animate_catvehicle`

    Higher level API for performing simulation on a circular trajectory of CAT Vehicle

    Parameters
    --------------
    num_vehicles: `integer`
         The number vehicles to be spawned in the simulation

    publish_rate: `double`
        The desired publish rate of the velocity of the vehicles' updated odometry to  update Gazebo simulation
    
    max_update_rate: `double`
        Max Update Rate to be set in Gazebo Physics Simulation
    
    time_step: `double`
        Time step for Gazebo Physics Simulation
    
    include_laser: `bool`
        Boolean flag to include/exclude laser model and laser plugin on the first vehicle model

    logdir: `string`
        Directory/Path where bag files and other data files recording simulation statistics will be saved.
    
    description: `string`
        Any human readable description of this simulation, the default is empty string, useful for annotating bag file name, will be saved  in the yaml file.
    
    Returns
    ----------
    `string`
        A list of data files is returned. We can specify only certain information to retrieved from saved bag files at the end of the simulation.

   '''

    print("Simulation beging")
   
    simConfig = {"circumference": circumference, "num_vehicle":  num_vehicles, "update_rate": publish_rate, "log_time": log_time, "max_update_rate": max_update_rate, "time_step": time_step, "include_laser": include_laser, "logdir": logdir, "description": description, "package_name":  package_name}
    
    
    print("Simulation Configuration: {}".format(simConfig))
    
    C = catvehicle(**simConfig)

    # Start a simulation
    bag = C.simulate(logdir=logdir)

    print("Bagfile recorded is {}".format(bag))

    gz_stat_file = C.gzstatsfile
    print("GZStat file is {}".format(gz_stat_file))
    GZ = GZStats(gz_stat_file)
    GZ.plotRTF()
    GZ.plotSimStatus()

    datafiles = []
    if bag is not None:
        Bag  = Bagplot(bag)
        datafiles = Bag.getDataFile(fileFilter="odom", msg_types = "odom")
        Bag.plot_timeseries(datafiles, 'PoseY', fileFilter='odoms')
        Bag.plot_topic_hz(datafiles)

        configfile =  C.bagfile[0:-4] + "/" + "simConfig.yaml"

        with open(configfile, 'w') as file:
            documents = yaml.dump(simConfig, file)

    time.sleep(6)
    pt.close('all')

    print("Simulation Ends. Datafiles saved are {}".format(datafiles))

    return datafiles

    
