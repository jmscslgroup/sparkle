#!/usr/bin/env python
# Initial Date: November 2019
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

import bagpy
import pandas as pd
import time, datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from matplotlib.pyplot import cm
import pickle
from bagpy import bagreader

'''
This class uses matlab's python engine and ROSBagReader class written in MATLAB
to create plots of required bag files

'''
color_long = ['#e6194b','#3cb44b','#ffe119','#0082c8','#f58231','#911eb4',\
              '#46f0f0','#f032e6','#d2f53c','#fabebe','#008080','#e6beff',\
              '#aa6e28','#800000','#aaffc3','#808000','#ffd8b1','#000080',\
              '#808080','#000000', "#FFFF00", "#1CE6FF", "#FF34FF", "#FF4A46", \
              "#008941", "#006FA6", "#A30059","#FFDBE5", "#7A4900", "#0000A6", \
              "#63FFAC", "#B79762", "#004D43", "#8FB0FF", "#997D87","#5A0007", \
              "#809693", "#6A3A4C", "#1B4400", "#4FC601", "#3B5DFF","#4A3B53", \
              "#FF2F80","#61615A", "#BA0900", "#6B7900", "#00C2A0", "#FFAA92", "#FF90C9", "#B903AA", "#D16100",\
              "#DDEFFF", "#000035", "#7B4F4B", "#A1C299", "#300018", "#0AA6D8", "#013349", "#00846F",\
              "#372101", "#FFB500", "#C2FFED", "#A079BF", "#CC0744", "#C0B9B2", "#C2FF99", "#001E09",\
              "#00489C", "#6F0062", "#0CBD66", "#EEC3FF", "#456D75", "#B77B68", "#7A87A1", "#788D66",\
              "#885578", "#FAD09F", "#FF8A9A", "#D157A0", "#BEC459", "#456648", "#0086ED", "#886F4C",\
              "#34362D", "#B4A8BD", "#00A6AA", "#452C2C", "#636375", "#A3C8C9", "#FF913F", "#938A81",\
              "#575329", "#00FECF", "#B05B6F", "#8CD0FF", "#3B9700", "#04F757", "#C8A1A1", "#1E6E00",\
              "#7900D7", "#A77500", "#6367A9", "#A05837", "#6B002C", "#772600", "#D790FF", "#9B9700",\
              "#549E79", "#FFF69F", "#201625", "#72418F", "#BC23FF", "#99ADC0", "#3A2465", "#922329",\
              "#5B4534", "#FDE8DC", "#404E55", "#0089A3", "#CB7E98", "#A4E804", "#324E72"]

def multi_plot_csv(csvlist, signal_name1, signal_name2,  description = "multi_csv_plot", savepath = "/home/ivory/CyverseData/ProjectSparkle/Consolidated Plot"):
    fig, ax = bagpy.create_fig(num_of_subplots=1)

    for i, f in enumerate(csvlist):
        df = pd.read_csv(f)
        if (signal_name1 in df.columns) and (signal_name2 in df.columns):
            ax[0].scatter(x = signal_name1, y=signal_name2, data=df, marker='*',  linewidth=0.3, s = 9, color=color_long[i])
        else:
            print("Specified component/signal {} unavailable in {}. Aborting! ".format(signal_name, f))
            return
    ax[0].legend(csvlist)
    ax[0].set_title("{}: Timseries plot for {}/{}".format(description, signal_name1, signal_name2), fontsize=16)
    ax[0].set_xlabel(signal_name1, fontsize=14)
    ax[0].set_ylabel(signal_name2, fontsize=14)

    current_fig = plt.gcf()
    dt_object = datetime.datetime.fromtimestamp(time.time())
    dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')
    fileToSave = savepath + "/" + dt + "_" + signal_name1.replace(".", "_") + "_" + signal_name2.replace(".", "_")

    with open(fileToSave + ".pickle", 'wb') as f:
        pickle.dump(fig, f) 
    current_fig.savefig(fileToSave + ".pdf", dpi = 100) 
    current_fig.savefig(fileToSave + ".png", dpi = 100) 
    plt.show()

def multi_plot(baglist, message_type , topic_name, signal_name, savepath = "/home/ivory/CyverseData/ProjectSparkle/Consolidated Plot"):
    '''
    Create a multi-plot of a topic and signal on the same plot

    For example, 6 bag files are sent as a `baglist`.One must specify type of messages
    among std, vel, wrench, odom. One desires to plot `linear.x` from topic `cmd_vel`, then, 
    `topic_name = "cmd_vel"` and `signal_name="linear.x"` is passed to the function. 

    Parameters
    -------------
    baglist: `list`

    A list of strings containing a list of full path of bagfiles

    message_type: `str`

    Message types among "std", "vel", "odometry", "laser", "wrench"

    topic_name: `str`

    signal_name: `str`

    savepath: `str`
    '''


    fig, ax = bagpy.create_fig()
    for bag in baglist:
        br = bagreader(bag)
        try:
            files = eval("br."+message_type + "_data()")
        except AttributeError:
            print("Unsupported type {}. Aborting.".format(message_type))
            return
    
    search_criteria = topic_name.replace("/", "-")

    required_files = []
    for file in files:
        if search_criteria in file:
            required_files.append(file)

    fig, ax = bagpy.create_fig(num_of_subplots=1)

    for i, f in enumerate(required_files):
        df = pd.read_csv(f)
        if signal_name in df.columns:
            ax.scatter(x = 'Time', y=signal_name, data=df, marker='*',  linewidth=0.3, s = 9, color=color_long[i])
        else:
            print("Specified component/signal {} unavailable in {}. Aborting! ".format(signal_name, f))
            return
    ax.legend(required_files)
    ax.set_title("Timseries plot for {}/{}".format(topic_name, signal_name), fontsize=16)
    ax.set_xlabel("Time", fontsize=14)
    ax.set_ylabel("Time", fontsize=14)

    current_fig = plt.gcf()
    dt_object = datetime.datetime.fromtimestamp(time.time())
    dt = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')
    fileToSave = savepath + "/" + dt + "_" + search_criteria + "_" + signal_name.replace(".", "_")

    with open(fileToSave + ".pickle", 'wb') as f:
        pickle.dump(fig, f) 
    current_fig.savefig(fileToSave + ".pdf", dpi = 100) 
    current_fig.savefig(fileToSave + ".png", dpi = 100) 
    plt.show()



def plot_topic_hz(csvfiles, window_size = 10, save = True):
    '''
    Plot the frequency / publish rateof timeseries data passed to the function
    '''            
    for datafile in csvfiles:
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

        fig, ax = bagpy.create_fig(num_of_subplots= 2)
        
        # Change the color and its transparency
        ti = Time[1:]
        ax[0].fill_between( Time[1:], frequency, color="skyblue", alpha=0.2)
        ax[0].plot(Time[1:], frequency, color="Slateblue", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax[0].set_axisbelow(True)
        ax[0].minorticks_on()
        ax[0].tick_params(axis="x", labelsize=14)
        ax[0].tick_params(axis="y", labelsize=14)
        ax[0].set_xlabel('Time Stamp', fontsize=14)
        ax[0].set_ylabel('Publish Rate/Frequency', fontsize=14)
        ax[0].set_title(datafile+ "\n" + "Publish Rate vs Time")

        # Change the color and its transparency
        ax[1].fill_between( Time[1:], std_dev, color="lightcoral", alpha=0.2)
        ax[1].plot(Time[1:], std_dev, color="crimson", alpha=0.6,  linestyle='-', linewidth='0.5', marker='.', markersize = 10)
        
        ax[1].set_axisbelow(True)
        ax[1].minorticks_on()
        ax[1].tick_params(axis="x", labelsize=14)
        ax[1].tick_params(axis="y", labelsize=14)
        ax[1].set_xlabel('Time Stamp', fontsize=14)
        ax[1].set_ylabel('Standard Deviation of  time diff', fontsize=14)
        ax[1].set_title(datafile+ "\n" + "Standard Deviation of time diff  vs Time")

        plt.tight_layout()
        if(save== True):
            current_fig = plt.gcf()
            fileToSave = datafile[0:-4]
            pickle.dump(fig,file(fileToSave + "topic_frequency.pickle",'w'))
            current_fig.savefig(fileToSave + "_topic_frequency.pdf", dpi = 300) 