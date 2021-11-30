#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  4 13:17:47 2021

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

Max_x_velocity_base = vel_linear_x_array.max()
Vel_vs_time_Lattice_Df = Total_vel_xy_Cum_Time
#removing inf values
Vel_vs_time_Lattice_Df.replace([np.inf, -np.inf], np.nan, inplace=True)
Vel_vs_time_Lattice_Df.dropna(inplace=True)

#Filter the Outliers Velocity

'''
#Using IQR 
Q1_filter = Vel_vs_time_Lattice_Df["Velocity XY"].quantile(0.25)
Q3_filter = Vel_vs_time_Lattice_Df["Velocity XY"].quantile(0.75)
IQR_filter = Q3_filter - Q1_filter
#print(IQR)
Outliers_fst_quartile = Vel_vs_time_Lattice_Df["Velocity XY"] < (Q1_filter - 1.5 * IQR_filter) 
Outliers_4th_quartile = Vel_vs_time_Lattice_Df["Velocity XY"] > (Q3_filter + 1.5 * IQR_filter)

remove_Outliers_filter = Vel_vs_time_Lattice_Df[~((Vel_vs_time_Lattice_Df < (Q1_filter - 10 * IQR_filter)) |(Vel_vs_time_Lattice_Df > (Q3_filter + 10 * IQR_filter))).any(axis=1)]

Filtered_Parameters_velocity_cum_time = remove_Outliers_filter
'''

#USING Z SCORE METHOD
Z_scores = stats.zscore(Vel_vs_time_Lattice_Df, nan_policy='omit')

abs_z_scores = np.abs(Z_scores)
filtered_entries = (abs_z_scores < 0.5).all(axis=1)
new_Velxy_cum_time_df = Vel_vs_time_Lattice_Df[filtered_entries]




plt.figure()
new_Velxy_cum_time_df.plot(x="Cummulative Time", y="Velocity XY")