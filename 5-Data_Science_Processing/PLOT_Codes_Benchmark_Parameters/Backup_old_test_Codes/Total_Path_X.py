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

Total_Path_Lattice_x = Total_path_x_traveled_Lattice
Total_Path_Astar_x = Total_path_x_traveled_Astar
Total_Path_Op_x = Total_path_x_traveled_Op
Total_PAth_Public_Road_x = Total_path_x_traveled_Public_Road


#Cummulative_pose_xy_Op 


Data_Dictionary = {'Op':Total_Path_Op_x, 'A*':Total_Path_Astar_x, 'Lattice':Total_Path_Lattice_x, 'Public_Road':Total_PAth_Public_Road_x }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_Total_Path_Df = pd.DataFrame(data_List)
Planners_Total_Path_Df.columns = ["Trajectory Planners", "Total Path X"]

sns.set_theme(style="whitegrid")
ax = sns.barplot(x= "Total Path X", y= "Trajectory Planners" , data = Planners_Total_Path_Df, orient= 'h')

ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

ax.invert_yaxis()