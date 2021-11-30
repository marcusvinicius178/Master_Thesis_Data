#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 13 20:23:07 2021

@author: autoware-auto-ros1
"""
import statistics
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd

#MEMORY USAGE (RAM)
#OP
OP_Memory = 5.7 #Gigabytes
Op_Memory_percentage = 9.7
#LATTICE
Lattice_Memory = 7.3 #Gigabytes
Lattice_Memory_percentage = 12,3
#ASTAR
Astar_Memory = 7,4 #Gigabytes
Astar_Memory_percentage = 12.6
#PUBLIC_ROAD
Public_Road_Memory =  10.9#Gigabytes
Public_Road_Memory_percentage = 18.5

#CPU USAGE

#OP
Open_Planner_CPU_cores = [30,53.5,33.3,33,39.4,79.6,61.5,34.3,95,94.1,41.1,48.1,42.7,44.1,27.3,50]
Op_CPU_median = statistics.mean(Open_Planner_CPU_cores)
Op_CPU_mean = round(statistics.median(Open_Planner_CPU_cores),3)
#Lattice
Lattice_CPU_cores = [14.4,58.6,31.2,71.4,37.9,34.4,99,23.6,86.7,95,96.1,48,47.3,65.6,39.6,80.8,]
Lattice_CPU_median = statistics.mean(Lattice_CPU_cores)
Lattice_CPU_mean = round(statistics.median(Lattice_CPU_cores),3)
#Astar
Astar_Planner_CPU_cores = [13,65.7,63.5,62.6,56.3,39.4,59.2,31.6,71.4,28.3,20.8,44.9,79.4,35.8,83.3,60]
Astar_CPU_median = statistics.mean(Astar_Planner_CPU_cores)
Astar_CPU_mean = round(statistics.median(Astar_Planner_CPU_cores),3)
#Public_Road
Public_Road_Planner_CPU_cores = [20.8,65.2,29.3,58.7,97,69.2,73.2,10.5,99,54.4,73.9,47.2,57.6,58.9,36.8,48.4]
Public_Road_CPU_median = statistics.mean(Public_Road_Planner_CPU_cores)
Public_Road_CPU_mean = round(statistics.median(Public_Road_Planner_CPU_cores),3)


f,axes = plt.subplots(2,1)


Data_Dictionary = {'Astar':Astar_CPU_median, 'Lattice':Lattice_CPU_median, 'Open Planner':Op_CPU_median, 'Public_Road':Public_Road_CPU_median }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_CPU_median_Df = pd.DataFrame(data_List)
Planners_CPU_median_Df.columns = ["Trajectory Planners", "Median of CPU Usage (%)"]


sns.set_theme(style="whitegrid")

ax = sns.barplot(x= "Median of CPU Usage (%)", y= "Trajectory Planners" , 
                 data = Planners_CPU_median_Df, 
                 palette=['Black', 'Red',  'Green','Blue'],
                 orient= 'h', ax=axes[0])


ax.bar_label(ax.containers[0])
ax.set(ylabel=None)

ax.invert_yaxis()



Data_Dictionary_mean = {'Astar':Astar_CPU_mean, 'Lattice':Lattice_CPU_mean, 'Open Planner':Op_CPU_mean, 'Public_Road':Public_Road_CPU_mean }
data_items_mean = Data_Dictionary_mean.items()
data_List_mean = list(data_items_mean)
Planners_CPU_mean_Df = pd.DataFrame(data_List_mean)
Planners_CPU_mean_Df.columns = ["Trajectory Planners", "Mean of CPU Usage (%)"]


sns.set_theme(style="whitegrid")

ax = sns.barplot(x= "Mean of CPU Usage (%)", y= "Trajectory Planners" , 
                 data = Planners_CPU_mean_Df, 
                 palette=['Black', 'Red',  'Green','Blue'],
                 orient= 'h', ax=axes[1])


ax.bar_label(ax.containers[0])
ax.set_xlabel('CPU Usage (%): Median (Up) | Mean (bottom)')
ax.set_ylabel('')

ax.invert_yaxis()


plt.show()




