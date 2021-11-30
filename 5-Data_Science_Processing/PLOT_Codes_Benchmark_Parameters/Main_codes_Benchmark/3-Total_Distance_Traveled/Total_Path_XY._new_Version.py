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
import pandas as pd


'''
Total_Path_Lattice = Total_path_xy_traveled_Lattice
Total_Path_Astar = Total_path_xy_traveled_Astar
Total_Path_Op = Total_path_xy_traveled_Op
Total_PAth_Public_Road = Total_path_xy_traveled_Public_Road

#The REAL path traveled by the planners is in block below, without filter the path to x minimum

Cummulative_pose_xy_Lattice
Cummulative_pose_xy_Astar
Cummulative_pose_xy_Op
Cummulative_pose_xy_Public_Road
'''
#Cummulative_pose_xy_Op 

Public_Road_Df = Filter_by_cum_x_Df_Public_Road
Astar_Df = Filter_by_cum_x_Df_Astar
Op_Df = Filter_by_cum_x_Df_Op
Lattice_Df = Filter_by_cum_x_Df_Lattice

Public_Road_last_x_pose = Filter_by_cum_x_Df_Public_Road['Cummulative X'].iloc[-1]
Astar_last_x_pose = Filter_by_cum_x_Df_Astar['Cummulative X'].iloc[-1]
Op_last_x_pose = Filter_by_cum_x_Df_Op['Cummulative X'].iloc[-1]
Lattice_last_x_pose = Filter_by_cum_x_Df_Lattice['Cummulative X'].iloc[-1]


Get_minimum_traveled_path_in_x = min(Public_Road_last_x_pose, Astar_last_x_pose, Op_last_x_pose, Lattice_last_x_pose)

#Filter the other Dataframes based on the limit of x pose = min = 78.8
Acceleration_List = Filtered_Parameters_Df #[(Filtered_Parameters_Df[['Acceleration X']] > 0).all(1)]


filtered_Df_Public_Road = Public_Road_Df[(Public_Road_Df[['Cummulative X']] < Get_minimum_traveled_path_in_x).all(1)]
filtered_Df_Astar = Astar_Df[(Astar_Df[['Cummulative X']] < Get_minimum_traveled_path_in_x).all(1)]
filtered_Df_Op = Op_Df[(Op_Df[['Cummulative X']] < Get_minimum_traveled_path_in_x).all(1)]
filtered_Df_Lattice = Lattice_Df[(Lattice_Df[['Cummulative X']] < Get_minimum_traveled_path_in_x).all(1)]

#Get The pow of the x and y

Df_Public_Road_xy = filtered_Df_Public_Road[['Delta X', 'DElta Y']].pow(2)
Df_Astar_xy = filtered_Df_Astar[['Delta X', 'DElta Y']].pow(2)
Df_Op_xy = filtered_Df_Op[['Delta X', 'DElta Y']].pow(2)
Df_Lattice_xy = filtered_Df_Lattice[['Delta X', 'DElta Y']].pow(2)

#Get sqrt and resultant xy

#Public_Road
Sum_Pose_xy_pow_Public_Road_df = Df_Public_Road_xy['Delta X'] + Df_Public_Road_xy['DElta Y']
Resultant_xy_Public_Road = Sum_Pose_xy_pow_Public_Road_df**(1/2)
Total_Distance_Traveled_Public_Road = sum(Resultant_xy_Public_Road)
#Astar
Sum_Pose_xy_pow_Astar_df = Df_Astar_xy['Delta X'] + Df_Astar_xy['DElta Y']
Resultant_xy_Astar = Sum_Pose_xy_pow_Astar_df**(1/2)
Total_Distance_Traveled_Astar = sum(Resultant_xy_Astar)
#Op
Sum_Pose_xy_pow_Op_df = Df_Op_xy['Delta X'] + Df_Op_xy['DElta Y']
Resultant_xy_Op = Sum_Pose_xy_pow_Op_df**(1/2)
Total_Distance_Traveled_Op = sum(Resultant_xy_Op)
#Lattice
Sum_Pose_xy_pow_Lattice_df = Df_Lattice_xy['Delta X'] + Df_Lattice_xy['DElta Y']
Resultant_xy_Lattice = Sum_Pose_xy_pow_Lattice_df**(1/2)
Total_Distance_Traveled_Lattice = sum(Resultant_xy_Lattice)




Data_Dictionary = {'A*':Total_Distance_Traveled_Astar, 'Lattice':Total_Distance_Traveled_Lattice, 'Open_Planner':Total_Distance_Traveled_Op, 'Public_Road':Total_Distance_Traveled_Public_Road }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_Total_Path_Df = pd.DataFrame(data_List)
Planners_Total_Path_Df.columns = ["Trajectory Planners", "Total Path"]
#clrs = ['Red' if Planners_Total_Path_Df['Trajectory Planners'][0] == 'Lattice' else 'blue'] 
sns.set_theme(style="whitegrid")
ax = sns.barplot(x= "Total Path", y= "Trajectory Planners" ,
                 data = Planners_Total_Path_Df,   
                 palette=['Black', 'Red', 'Green', 'Blue'],
                 orient= 'h')

ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

ax.invert_yaxis()


#for bar in ax.patches:
#    a = bar._alias_map.keys
    

