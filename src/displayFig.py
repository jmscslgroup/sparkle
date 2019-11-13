#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.
import pandas as pd
import sys, math, time, datetime
import matplotlib.pyplot as pt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from matplotlib.pyplot import cm
import pickle


# Load figure from disk and display
fileName = '/home/ivory/VersionControl/catvehicle_ws/src/sparkle/src/Consolidated Plot/2019-11-11-21-16-47-778876_odom.pickle'
fig_handle = pickle.load(open(fileName,'rb'))
pt.show()