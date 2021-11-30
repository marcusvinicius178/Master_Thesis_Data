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
Lattice_Memory_percentage = 12.3
#ASTAR
Astar_Memory = 7.4 #Gigabytes
Astar_Memory_percentage = 12.6
#PUBLIC_ROAD
Public_Road_Memory =  10.9#Gigabytes
Public_Road_Memory_percentage = 18.5

#FREE MEMORY
Total_Used = Op_Memory_percentage + Lattice_Memory_percentage+Astar_Memory_percentage+Public_Road_Memory_percentage
Free_Memory = float(100) - Total_Used

# Creating dataset
cars = ['Open Planner', 'Lattice', 'Astar', 
        'Public Road']
  
data = [OP_Memory, Lattice_Memory, Astar_Memory, Public_Road_Memory]
data_memory_percentage = [Op_Memory_percentage, Lattice_Memory_percentage,Astar_Memory_percentage,Public_Road_Memory_percentage ]
data_string = [str(ele) for ele in data]
  
# Creating explode data
explode = (0.1, 0.0, 0.2, 0.3)
  
# Creating color parameters
colors = ( "Green", "Red", "Black", "Blue")
  
# Wedge properties
wp = { 'linewidth' : 1, 'edgecolor' : "green" }
  

def func(p,allvalues, allvalues2):
    absolute_percentage = p* (np.sum(data_memory_percentage))/100
    absolute = p*(np.sum(data))/100
    return '{:.2f}%\n({:.2f} Gb)'.format(absolute_percentage, absolute)

  
# Creating plot
fig, ax = plt.subplots(figsize =(10, 7))
wedges, texts, autotexts = ax.pie(data, 
                                  autopct = lambda p: func(p,data, data_memory_percentage),
                                  explode = explode, 
                                  labels = cars,
                                  shadow = True,
                                  colors = colors,
                                  startangle = 90,
                                  wedgeprops = wp,
                                  textprops = dict(color ="Black",
                                  fontsize=13))
  
# Adding legend
ax.legend(wedges, cars,
          title ="Trajectory Planners",
          loc ="lower left",
          bbox_to_anchor =(1, 0, 0.5, 1))
  
plt.setp(autotexts, size = 17, weight ="bold", color="White")
ax.set_title("RAM USAGE (Available = 58.9 Gb)", fontsize = 20)
  
# show plot
plt.show()