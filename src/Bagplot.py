#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import signal
import matlab.engine
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

class Bagplot(object):
    '''
    __init__ takes bag file name (either relative path or full path) and optionally path to ROSBagReader in your system
    __init__ also starts MATLAB's python engine

    '''
    def __init__(self, bagfile, ROSBagReaderPath='/home/ivory/VersionControl/Jmscslgroup/ROSBagReader'):
        self.bagfile = bagfile
        self.ROSBagReaderPath = ROSBagReaderPath
        
        # Start the matlab engine
        self.engine = matlab.engine.start_matlab()
        
        # Add path of ROSBagReader to MATLAB enviornment so that MATLAB can find ROSBagReader definition
        self.engine.addpath(ROSBagReaderPath)

        self.BagReader = self.engine.ROSBagReader(bagfile)

    '''
        Returns the list of Data files according to the file filter

    '''
    def getDataFile(self, fileFilter="magna-odom", msg_types = "odom"):
        self.engine.workspace["BagReader"] = self.BagReader
        
        datafiles = []
        if msg_types == "odom":
            datafiles = self.engine.eval("BagReader.extractOdometryData()")

        # I want data files only of time *-odom
        for file in datafiles:
            
            if fileFilter not in file:
                datafiles.remove(file)

        return 
        
    '''
    Plot specific single attributes from the array of files passed
    '''
    def plot_timeseries(self, datafiles, attribute, fileFilter='_'):
        Title = self.bagfile[0:-4]
        plot_timeseries(datafiles, attribute, fileFilter, Title)

        

'''
Plot specific single attributes from the array of files passed
'''
def plot_timeseries(datafiles, attribute, fileFilter='_',  Title=None):


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

    pt.rcParams["figure.figsize"] = (16,8)
    params = {'legend.fontsize': 10, 'legend.handlelength': 2, 'legend.loc': 'upper right'}
    pt.rcParams.update(params)
    pt.rcParams["font.family"] = "Times New Roman"
    fig = pt.figure()
    ax = fig.add_subplot(1,1,1)
    ax.set_axisbelow(True)
    ax.minorticks_on()
    ax.tick_params(axis="x", labelsize=16)
    ax.tick_params(axis="y", labelsize=16)
    pt.grid(True)
    ax.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
    ax.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
    ax.set_xlabel('Time', fontsize=16)
    ax.set_ylabel(attribute, fontsize=16)
    
    colors=cm.rainbow(np.linspace(0,1,len(XArray)))

    for i in range(0, len(XArray)):
        c=colors[i]
        pt.plot(TimeArray[i] - TimeArray[i][0], XArray[i], color =c, linewidth = 1, linestyle=None, marker='.', markersize = 5)
    ax.legend(FileName)


    ax.set_title(Title)
    current_fig = pt.gcf()
    # pt.show()
    dt_object = datetime.datetime.fromtimestamp(time.time())
    dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')
    fileNameToSave =Title + "/" + dt + "_" + fileFilter

    pickle.dump(fig,file(fileNameToSave + ".pickle",'w'))
    current_fig.savefig(fileNameToSave + ".pdf", dpi = 300)


   