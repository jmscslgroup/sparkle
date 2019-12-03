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

class GZStats(object):
    '''
    __init__ takes  file name of gz stats dump (either relative path or full path) 

    '''
    def __init__(self, statfile, ROSBagReaderPath='/home/ivory/VersionControl/Jmscslgroup/ROSBagReader'):
        self.statfile = statfile


def parseRTF(self, filename='Circle_Test_n_20_updateRate_1_2019-12-02-13-13-35_gzStats.txt'):
    dataframe = pd.read_csv(filename, sep=' ' , header=None, names=["Factor", "SimTime", "RealTime", "SimStatus"])