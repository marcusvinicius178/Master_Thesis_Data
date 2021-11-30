#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 16:01:01 2021

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

Total_Path_Lattice_y = Total_path_y_traveled_Lattice
Total_Path_Astar_y = Total_path_y_traveled_Astar
Total_Path_Op_y = Total_path_y_traveled_Op
Total_PAth_Public_Road_y = Total_path_y_traveled_Public_Road


#Cummulative_pose_xy_Op 


Data_Dictionary = { 'A*':Total_Path_Astar_y, 'Lattice':Total_Path_Lattice_y, 'Op':Total_Path_Op_y, 'Public_Road':Total_PAth_Public_Road_y }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_Total_Path_Df = pd.DataFrame(data_List)
Planners_Total_Path_Df.columns = ["Trajectory Planners", "Lateral Path Distortion Region"]

sns.set_theme(style="whitegrid")
ax = sns.barplot(x= "Trajectory Planners", y= "Lateral Path Distortion Region",  
                 data = Planners_Total_Path_Df,
                 palette=['Black', 'Red','Green', 'Blue'],
                 orient= 'v')

ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

#ax.invert_yaxis()