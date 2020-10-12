#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved

from .circle import circle
from .catvehicle import catvehicle
from .lane import lane
from .plot_util import plot_topic_hz
from bagpy import bagreader
from .gzstats import gzstats
import pandas as pd
import  time
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import yaml
from operator import add

def animate_lane(vehicle_spacing,  n_vehicles, leader_vel,  publish_rate, max_update_rate, time_step, log_time, include_laser, logdir, description=""):
    '''
    Sparkle Simulation API: `animate_lane`

    Higher level API for performing simulation on a straight lanes

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
    print("Lane Simulation Begin")
    simConfig = {"vehicle_spacing": vehicle_spacing, "n_vehicles":  n_vehicles, "update_rate": publish_rate, "log_time": log_time, "max_update_rate": max_update_rate, "time_step": time_step, "include_laser": include_laser, "logdir": logdir, "description": description}
    
    print("Simulation Configuration: {}".format(simConfig))
    L = lane(**simConfig)

    # Start simulation
    bag = L.simulate(leader_vel , logdir=logdir)
    print("Bagfile recorded is {}".format(bag))
    return bag

def animate_circle(circumference, n_vehicles, leader_vel,  publish_rate, max_update_rate, time_step, log_time, include_laser, logdir, description=""):
    '''
    Sparkle Simulation API: `animate_circle`

    Higher level API for performing simulation on a circular trajectory

    Parameters
    --------------
    n_vehicles: `integer`
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

    print("Simulation begin")
    simConfig = {"circumference": circumference, "n_vehicles":  n_vehicles, "update_rate": publish_rate, \
        "log_time": log_time, "max_update_rate": max_update_rate, "time_step": time_step, \
            "include_laser": include_laser, "logdir": logdir, "description": description}
    
    print("Simulation Configuration: {}".format(simConfig))
    C = circle(**simConfig)

    # Start a simulation
    bag = C.simulate(leader_vel, logdir=logdir)
    print("Bagfile recorded is {}".format(bag))

    C.analyze()
    # gz_stat_file = C.gzstatsfile
    # print("GZStat file is {}".format(gz_stat_file))
    # GZ = gzstats(gz_stat_file)
    # GZ.plotRTF()
    # GZ.plotSimStatus()

    csvfiles = []
    if bag is not None:
        br = bagreader(bag)
        csvfiles = br.odometry_data()
        plot_topic_hz(csvfiles)
        configfile =  C.bagfile[0:-4] + "/" + "simConfig.yaml"

        with open(configfile, 'w') as file:
            documents = yaml.dump(simConfig, file)
    time.sleep(6)
    pt.close('all')
    print("Simulation Ends.")
    return bag

def circle_catvehicle(package_name, circumference, n_vehicles, leader_vel, publish_rate, max_update_rate, time_step, log_time, include_laser, logdir, description="", plot=False, **kwargs):
    '''
    Sparkle Simulation API: `circle_catvehicle`

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
   
    simConfig = {"circumference": circumference, "n_vehicles":  n_vehicles, "update_rate": publish_rate, "log_time": log_time, "max_update_rate": max_update_rate, "time_step": time_step, "include_laser": include_laser, "logdir": logdir, "description": description, "package_name":  package_name}
    
    
    print("Simulation Configuration: {}".format(simConfig))
    
    C = catvehicle(**simConfig)

    # Start a simulation
    bag = C.simulate(leader_vel = leader_vel, logdir=logdir)

    print("Bagfile recorded is {}".format(bag))

    gz_stat_file = C.gzstatsfile
    print("GZStat file is {}".format(gz_stat_file))
    GZ = gzstats(gz_stat_file)
    GZ.dataframe.to_csv(gz_stat_file[0:-4]+".csv")

    if plot:
        GZ.plotRTF()
        GZ.plotSimStatus()

        if bag is not None:
            br = bagreader(bag)
            csvfiles = br.odometry_data()
            plot_topic_hz(csvfiles)

    configfile =  C.bagfile[0:-4] + "/" + "simConfig.yaml"

    with open(configfile, 'w') as file:
        documents = yaml.dump(simConfig, file)

    time.sleep(6)
    pt.close('all')

    print("Simulation Ends.")
    return bag

    
