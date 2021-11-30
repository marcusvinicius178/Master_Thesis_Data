#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 21:49:14 2021

@author: autoware-auto-ros1
"""

from Power_Consumption_Astar_Mechanic_Energy import *
from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_Public_Road_Mechanic_Energy import *
from Power_Consumption_OP_Mechanic_Energy import *
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from scipy import stats
import pandas as pd

Total_Time_Lattice = Trip_seconds_Lattice
Total_Time_Astar = Trip_seconds_Astar
Total_Time_Op = Trip_seconds_Op
Total_Time_Public_Road = Trip_seconds_Public_Road


#Cummulative_pose_xy_Op 


Data_Dictionary = {'A*':Total_Time_Astar, 'Lattice':Total_Time_Lattice, 'Op':Total_Time_Op, 'Public_Road':Total_Time_Public_Road }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_Total_Time_Df = pd.DataFrame(data_List)
Planners_Total_Time_Df.columns = ["Trajectory Planners", "Time Consumption"]

sns.set_theme(style="whitegrid")
ax = sns.barplot( x= "Trajectory Planners", y= "Time Consumption" , 
                 data = Planners_Total_Time_Df,
                 palette=['Black', 'Red', 'Green', 'Blue'],
                 orient= 'v')

ax.set(ylabel='Time Consumption (s)')

ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

#ax.invert_yaxis()