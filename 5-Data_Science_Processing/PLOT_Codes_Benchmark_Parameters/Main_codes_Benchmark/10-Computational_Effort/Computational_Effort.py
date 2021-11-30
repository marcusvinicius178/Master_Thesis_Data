#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 13 20:23:07 2021

@author: autoware-auto-ros1
"""
import statistics
import matplotlib.pyplot as plt
import numpy as np

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
Op_CPU_mean = statistics.median(Open_Planner_CPU_cores)
#Lattice
Lattice_CPU_cores = [14.4,58.6,31.2,71.4,37.9,34.4,99,23.6,86.7,95,96.1,48,47.3,65.6,39.6,80.8,]
Lattice_CPU_median = statistics.mean(Open_Planner_CPU_cores)
Lattice_CPU_mean = statistics.median(Open_Planner_CPU_cores)
#Astar
Astar_Planner_CPU_cores = [13,65.7,63.5,62.6,56.3,39.4,59.2,31.6,71.4,28.3,20.8,44.9,79.4,35.8,83.3,60]
Astar_CPU_median = statistics.mean(Open_Planner_CPU_cores)
Astar_CPU_mean = statistics.median(Open_Planner_CPU_cores)
#Public_Road
Public_Road_Planner_CPU_cores = [20.8,65.2,29.3,58.7,97,69.2,73.2,10.5,99,54.4,73.9,47.2,57.6,58.9,36.8,48.4]
Public_Road_CPU_median = statistics.mean(Open_Planner_CPU_cores)
Public_Road_CPU_mean = statistics.median(Open_Planner_CPU_cores)


X = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10' , '11', '12', '13', '14', '15', '16' ]
X_axis = np.arange(len(X))

n = 16
r = np.arange(n)
width= 0.25

plt.bar(r, Astar_Planner_CPU_cores, color='Black', width=width, edgecolor='Black', label='Astar')
plt.bar(r + width, Lattice_CPU_cores, color='Red', width=width, edgecolor='Black', label='Lattice'  )
plt.bar(r + 2*width, Open_Planner_CPU_cores, color='Green', width=width, edgecolor='Black', label='Open Planner')
plt.bar(r + 3*width, Public_Road_Planner_CPU_cores, color='Blue', width=width, edgecolor='Black', label='Public Road')




'''
ax.bar(labels, Open_Planner_CPU_cores, label='Open Planner', color="Green")
ax.bar(labels, Lattice_CPU_cores, label='Lattice', color="Red")
ax.bar(labels, Astar_Planner_CPU_cores, label='Astar', color="Black")
ax.bar(labels, Public_Road_Planner_CPU_cores, label='Public Road', color="Blue")
'''
plt.ylabel('CPU: Usage (%)')
plt.xlabel('Threads N°')
plt.xticks(r + width/2, X)
plt.legend(prop={'size': 8})
plt.show()






