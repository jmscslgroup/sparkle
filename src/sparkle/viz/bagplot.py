#!/usr/bin/env python
# Initial Date: November 2019
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import signal
import bagpy
import pandas as pd
import sys, math, time, datetime
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from matplotlib.pyplot import cm
import pickle

'''
This class uses matlab's python engine and ROSBagReader class written in MATLAB
to create plots of required bag files

'''

class bagplot(object):
    '''
    Plotting and analysis of Bag files saved at the end of the simulation

    Parameters
    ------------
    
    bagfile: `string`
        Full path of the bafile

    '''
    def __init__(self, bagfile):
        self.bagfile = bagfile        
        self.bagreader = bagpy.bagreader(bagfile)

    def datafile(self, fileFilter="magna-odom", msg_types = "odom"):
        '''
        Returns the list of Data files according to the file filter
        '''
        datafiles = []
        if msg_types == "odom":
            datafiles = self.bagreader.odometry_data()

        # I want data files only of time *-odom
        for file in datafiles:
            
            if fileFilter not in file:
                datafiles.remove(file)

        return datafiles
        
    def plot_timeseries(self, datafiles, attribute, fileFilter='_'):
        '''
        Plot specific single attributes from the array of files passed
        '''
        Title = self.bagfile[0:-4]

        plot_ts(datafiles, attribute, fileFilter, Title)

    @staticmethod
    def plot_topic_hz(datafiles, window_size = 10, save = True):
        '''
        Plot the frequency / publish rateof timeseries data passed to the function
        '''            
        datafile = datafiles[0]
        Time = []
        if ".csv" in datafile:
            dataframe = pd.read_csv(datafile)
            Time = dataframe["Time"]

        time_diff = np.diff(Time)
        time_length = len(time_diff)

        assert time_length > window_size, "Number of messages must be greater than the window size"

        frequency = [] # Hz
        std_dev = [] # seconds

        for i in range(0,  window_size):
            frequency.append(0)
            std_dev.append(0)

        for i in range(window_size,  time_length):
            mean = np.mean(time_diff[i-window_size:i])
            rate = 1./mean if mean > 0. else 0.0
            frequency.append(rate)
            std = np.std(time_diff[i-window_size:i])
            std_dev.append(std)

        pt.style.use('seaborn')
        pt.rcParams["font.family"] = "Roboto"
        pt.rcParams["figure.figsize"] = (18,12)
        params = {'legend.fontsize': 16, 'legend.handlelength': 2, 'legend.loc': 'upper left'}
        pt.rcParams.update(params)
        fig, (ax1, ax2) = pt.subplots(2, 1)
        
        # Change the color and its transparency
        ti = Time[1:]
        ax1.fill_between( Time[1:], frequency, color="skyblue", alpha=0.2)
        ax1.plot(Time[1:], frequency, color="Slateblue", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax1.set_axisbelow(True)
        ax1.minorticks_on()
        ax1.tick_params(axis="x", labelsize=14)
        ax1.tick_params(axis="y", labelsize=14)
        ax1.set_xlabel('Time Stamp', fontsize=14)
        ax1.set_ylabel('Publish Rate/Frequency', fontsize=14)
        ax1.set_title(datafile+ "\n" + "Publish Rate vs Time")

        # Change the color and its transparency
        ax2.fill_between( Time[1:], std_dev, color="lightcoral", alpha=0.2)
        ax2.plot(Time[1:], std_dev, color="crimson", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax2.set_axisbelow(True)
        ax2.minorticks_on()
        ax2.tick_params(axis="x", labelsize=14)
        ax2.tick_params(axis="y", labelsize=14)
        ax2.set_xlabel('Time Stamp', fontsize=14)
        ax2.set_ylabel('Standard Deviation of  time diff', fontsize=14)
        ax2.set_title(datafile+ "\n" + "Standard Deviation of time diff  vs Time")

        pt.tight_layout()
        if(save== True):
            current_fig = pt.gcf()
            fileToSave = datafile[0:-4]
            pickle.dump(fig,file(fileToSave + "topic_frequency.pickle",'w'))
            current_fig.savefig(fileToSave + "_topic_frequency.pdf", dpi = 300) 

        #pt.show()

'''
Plot specific single attributes from the array of files passed
'''
def plot_ts(datafiles, attribute, fileFilter='_',  Title=None):

    if Title is None:
        Title = "Untitled"
    TimeArray = []
    XArray = []
    FileName   = []
    for f in datafiles:
        if ".csv" in f:
            data_frame = pd.read_csv(f)
            Time = data_frame["Time"]
            X = data_frame[attribute]
            TimeArray.append(Time)
            XArray.append(X)
            FileName.append(f)

    pt.style.use('seaborn')
    pt.rcParams["figure.figsize"] = (15,10)
    pt.rcParams[ 'font.family'] = 'Roboto'
    pt.rcParams[ 'font.weight'] = 'bold'
    params = {'legend.fontsize': 7, 'legend.handlelength': 2, 'legend.loc': 'upper right'}
    pt.rcParams.update(params)
    fig,ax = pt.subplots(1)
    colors=cm.rainbow(np.linspace(0,1,len(XArray)))

    for i in range(0, len(XArray)):
        c=colors[i]
        pt.plot(TimeArray[i] - TimeArray[i][0], XArray[i], color =c, linewidth = 1, linestyle=None, marker='.', markersize = 5)
    ax.legend(FileName)
    ax.set_xlabel('Time', fontsize=12)
    ax.set_ylabel(attribute, fontsize=12)

    ax.set_title(Title, fontsize=10)
    current_fig = pt.gcf()
    # pt.show()
    dt_object = datetime.datetime.fromtimestamp(time.time())
    dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')
    fileNameToSave = Title + "/" + dt + "_" + fileFilter

    pickle.dump(fig,file(fileNameToSave + ".pickle",'w'))
    current_fig.savefig(fileNameToSave + ".pdf", dpi = 300)


   
