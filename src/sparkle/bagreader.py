#!/usr/bin/env python
# Initial Date: January 2020
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved

import subprocess
import yaml
import rosbag

import numpy  as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sea
import plotnine as pn

class bagreader:
    '''
    `bagreader` class provides API to read rosbag files in an effective easy manner with significant hassle.
    This class is reimplementation of its MATLAB equivalent that can be found at https://github.com/jmscslgroup/ROSBagReader

    Parameters
    ----------------
    bagfile: `string`
        Bagreader constructor takes name of a bag file as an  argument. name of the bag file can be provided as the full  qualified path, relative path or just the file name.

    Attributes
    --------------
    bagfile: `string`
        Full path of the bag  file, e.g /home/ece446/2019-08-21-22-00-00.bag
    filename; `string`
        Name of the bag file, e.g. 2019-08-21-22-00-00.bag
    dir: `string`
        Directory where bag file is located
    reader: `rosbag.Bag`
        rosbag.Bag object that 

    topic: `pandas dataframe`
        stores the available topic from bag file being read as a table
    n_messages: `integer`
        stores the number of messages
    message_types:`list`, `string`
        stores all the available message types
    datafolder: `string`
        stores the path/folder where bag file is present - may be relative to the bag file or full-qualified path.

        E.g. If bag file is at /home/ece446/2019-08-21-22-00-00.bag, then datafolder is /home/ece446/2019-08-21-22-00-00/

    message_dictionary: `dictionary`
        message_dictionary will be a python dictionary to keep track of what datafile have been generated mapped by types

    graycolordark: `tuple`
        dark gray color for timeseries plots

    graycolorlight: `tuple`
        light gray color for timeseries plots
    
    linecolor: `tuple`
        a set of line color for timeseries plots
    markercolor: `tuple`
        a set of marker color for timeseries plots

    '''

    def __init__(self, bagfile):
        self.bagfile = bagfile
        
        slashindices = find(bagfile, '/')
        
        # length of slashindices list will be zero if a user has pass only bag file name , e.g. 2020-03-04-12-22-42.bag
        if  len(slashindices) > 0:
            self.filename =bagfile[slashindices[-1]:]
            self.dir = bagfile[slashindices[0]:slashindices[-1]]
        else:
            self.filename = bagfile
            self.dir = './'

        self.reader = rosbag.Bag(self.bagfile)

        info = self.reader.get_type_and_topic_info() 
        self.topic_tuple = info.topics.values()
        self.topics = info.topics.keys()

        self.message_types = []
        for t1 in self.topic_tuple: self.message_types.append(t1.msg_type)

        self.n_messages = []
        for t1 in self.topic_tuple: self.message_types.append(t1.message_count)

        self.frequency = []
        for t1 in self.topic_tuple: self.frequency.append(t1.frequency)

        df = pd.DataFrame(list(zip(self.topics, self.message_types, self.n_messages, self.frequency)), columns=['Topics', 'Types', 'Message Count', 'Frequency'])

        self.start_time = self.reader.get_start_time()
        self.end_time = self.reader.get_end_time()



        

    def laser_data(self, **kwargs):
        pass

    def vel_data(self, **kwargs):
        pass

    def std_data(self, **kwargs):
        pass

    def compressed_images(self, **kwargs):
        pass

    def odometry_data(self, **kwargs):
        pass

    def wrench_data(self, **kwargs):
        pass

    def  clock_data(self, **kwargs):
        pass

    def pointcloud_data(self, **kwargs):
        pass

    def ts_plot_vel(self):
        pass

    def ts_plot_std(self):
        pass

    def ts_plot_odometry(self):
        pass

    def ts_wrench_vel(self):
        pass

    def ts_plot_vel(self):
        pass
    
    def animate_laser(self):
        pass

    def animate_pointcloud(self):
        pass

def find(s, ch):
    '''
    Function `find` returns indices all the occurence of `ch` in `s` 

    Parameters
    -------------
    s: `string`
        String or a setence where to search for occurrences of the character `ch`

    s: `char`
        Character to look for

    Returns
    ---------
    `list`
        List of indices of occurrences of character `ch` in the string `s`.

    '''
    return [i for i, ltr in enumerate(s) if ltr == ch]



    
    
    