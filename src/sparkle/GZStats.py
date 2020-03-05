#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import signal
import pandas as pd
import sys, math, time, datetime
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from matplotlib.pyplot import cm
import pickle


class GZStats(object):
    '''
    __init__ takes  file name of gz stats dump (either relative path or full path) 

    '''
    def __init__(self, statfile='Circle_Test_n_20_updateRate_1_2019-12-02-13-13-35_gzStats.txt'):
        self.statfile = statfile
        
        dataframe = pd.read_csv(self.statfile, sep=' ' , header=None, names=["Factor", "SimTime", "RealTime", "SimStatus"])
        
        # delete the last two rows, they may be incomplete
        dataframe.drop(dataframe.tail(2).index,inplace=True)
        df = dataframe.replace(r'\w+\[([\S\.]+)\]', r'\1', regex=True)
        df['Factor'] = df['Factor'].astype('float')
        df['SimTime'] = df['SimTime'].astype('float')
        df['RealTime'] = df['RealTime'].astype('float')
        df['SimStatus'] = df['SimStatus'].apply(lambda x: self._status(x))

        

        self.dataframe = df

        # Mean Real Time Factor throught the simulation
        self.rtf_avg = round(np.mean(self.dataframe['Factor']), 5)

        # Mean Standard Deviation of RTF throughout the Simulation
        self.rtf_std =  round(np.std(self.dataframe['Factor']), 5)

        # Pause Ratio is the percentage of the time when Simulation stayed pause
        self.pause_ratio = round(self._calcPausePercentage(self.dataframe['SimStatus']), 5)

    '''
    Private function to use in Sim Status Lambda
    '''
    def _status(self, x):
        if(x == 'F'):
            x = 0.0
        elif(x == 'T'):
            x = 1.0
        else:
            x = None
        return x


    '''
    Private function to calculate pause percentage
    '''
    def _calcPausePercentage(self, data):
        return (sum(data)/len(data))

    def plotRTF(self, save=True):
        SimTime = self.dataframe['SimTime']
        RealTime = self.dataframe['RealTime']
        Factor = self.dataframe['Factor']

        pt.style.use('seaborn')
        pt.rcParams["figure.figsize"] = (18,12)
        params = {'legend.fontsize': 16, 'legend.handlelength': 2, 'legend.loc': 'upper right'}
        pt.rcParams.update(params)
        fig, (ax1, ax2) = pt.subplots(2, 1)
        
        # Change the color and its transparency
        ax1.fill_between( SimTime, Factor, color="skyblue", alpha=0.2)
        ax1.plot(SimTime, Factor, color="Slateblue", alpha=0.6, linestyle='-', linewidth='1', marker='.', markersize = 3)
        
        ax1.set_axisbelow(True)
        ax1.minorticks_on()
        ax1.tick_params(axis="x", labelsize=16)
        ax1.tick_params(axis="y", labelsize=16)
        pt.grid(True)
        ax1.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
        ax1.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
        ax1.set_xlabel('Sim Time', fontsize=16)
        ax1.set_ylabel('Real Time Factor', fontsize=16)
        ax1.legend(['Real time factor average: ' + str(self.rtf_avg) + ', std: ' + str(self.rtf_std)])
        ax1.set_title( self.statfile[0:-4]+ "\n " + "Real Time Factor vs Sim Time")
        
        # Change the color and its transparency
        ax2.fill_between( RealTime, Factor, color="lightcoral", alpha=0.2)
        ax2.plot(RealTime, Factor, color="crimson", alpha=0.6, linestyle='-', linewidth='1', marker='.', markersize = 3)
        
        ax2.set_axisbelow(True)
        ax2.minorticks_on()
        ax2.tick_params(axis="x", labelsize=16)
        ax2.tick_params(axis="y", labelsize=16)
        ax2.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
        ax2.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
        ax2.set_xlabel('Real Time', fontsize=16)
        ax2.set_ylabel('Real Time Factor', fontsize=16)
        ax2.legend(['Real time factor average: ' + str(self.rtf_avg) + ', std: ' + str(self.rtf_std)])
        ax2.set_title( self.statfile[0:-4]+ "\n" + "Real Time Factor vs Real Time")

 

        if(save== True):
            current_fig = pt.gcf()
            fileToSave = self.statfile[0:-4]
            pickle.dump(fig,file(fileToSave + "_RTF.pickle",'w'))
            current_fig.savefig(fileToSave + "_RTF.pdf", dpi = 300) 

        #pt.show()
        
    def plotSimStatus(self, save=True):
        SimTime = self.dataframe['SimTime']
        RealTime = self.dataframe['RealTime']
        SimStatus = self.dataframe['SimStatus']

        pt.style.use('seaborn')
        pt.rcParams["figure.figsize"] = (18,12)
        params = {'legend.fontsize': 16, 'legend.handlelength': 2, 'legend.loc': 'upper left'}
        pt.rcParams.update(params)
        fig, (ax1, ax2) = pt.subplots(2, 1)
        
        # Change the color and its transparency
        ax1.plot(SimTime, SimStatus, color="Slateblue", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax1.set_axisbelow(True)
        ax1.minorticks_on()
        ax1.tick_params(axis="x", labelsize=16)
        ax1.tick_params(axis="y", labelsize=16)
        pt.grid(True)
        ax1.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
        ax1.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
        ax1.set_xlabel('Sim Time', fontsize=16)
        ax1.set_ylabel('Gazebo Sim Pause Status', fontsize=16)
        ax1.legend(['Pause percentage: ' + str(self.pause_ratio*100)+ '%'])
        ax1.set_title( self.statfile[0:-4]+ "\n" + "Sim Status (Pause/Unpause) vs Sim Time")
        
        # Change the color and its transparency
        ax2.plot(RealTime, SimStatus, color="crimson", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax2.set_axisbelow(True)
        ax2.minorticks_on()
        ax2.tick_params(axis="x", labelsize=16)
        ax2.tick_params(axis="y", labelsize=16)
        ax2.grid(which='major', linestyle='-', linewidth='0.5', color='skyblue')
        ax2.grid(which='minor', linestyle=':', linewidth='0.25', color='dimgray')
        ax2.set_xlabel('Real Time', fontsize=16)
        ax2.set_ylabel('Gazebo Sim Pause Status', fontsize=16)
        ax2.legend(['Pause percentage: ' + str(self.pause_ratio*100) + '%'])
        ax2.set_title( self.statfile[0:-4]+ "\n" + "Sim Status (Pause/Unpause) vs Real Time")
 

        if(save== True):
            current_fig = pt.gcf()
            fileToSave = self.statfile[0:-4]
            pickle.dump(fig,file(fileToSave + "SimStatus.pickle",'w'))
            current_fig.savefig(fileToSave + "_SimStatus.pdf", dpi = 300) 

       # pt.show()
        