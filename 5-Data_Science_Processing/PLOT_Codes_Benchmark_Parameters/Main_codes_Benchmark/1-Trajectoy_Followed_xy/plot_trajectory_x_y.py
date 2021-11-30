#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 22 16:17:42 2021

@author: autoware-auto-ros1
"""
import pandas as pd
import matplotlib.pyplot as plt

from Power_Consumption_Astar_Mechanic_Energy import *
from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_OP_Mechanic_Energy import *
from Public_Road_data_to_plot_vel_pose_steer import * #just contain more timestamp points than Power_Consumption_Public_Road_Mechanic_Energy
#because of a merge required step between differente csv data.

pose_x_Astar = pose_x_array_Astar
pose_y_Astar = pose_y_array_Astar

pose_x_Lattice = pose_x_array_Lattice 
pose_y_Lattice = pose_y_array_Lattice


pose_x_Op = pose_x_array_Op
pose_y_Op = pose_y_array_Op

pose_x_Public_Road = pose_x_array_Public_Road
pose_y_Public_Road = pose_y_array_Public_Road


class Trajectory:
    
    def path (self, pose_x, pose_y, N_waypoints, planner_name):
        
        self.planner_name = planner_name
        self.n_of_waypoints = str(N_waypoints)
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.name = self.planner_name+':' +' ' +self.n_of_waypoints+' '+ 'waypoints'
        
        
        self.relative_pose_x = []
        for i in range(len(self.pose_x)):
            value_x =  abs(self.pose_x.iloc[i] - self.pose_x.iloc[0])
            self.relative_pose_x.append(value_x)
            
        self.relative_pose_y = []
        for j in range(len(self.pose_y)):
            value_y =  abs(self.pose_y.iloc[j] - self.pose_y.iloc[0])
            self.relative_pose_y.append(value_y)
            
        
        self.pose_df = pd.DataFrame({'Pose_x':self.relative_pose_x, self.name:self.relative_pose_y})
            
            
Astar = Trajectory()
Astar.path(planner_name = 'Freespace Astar', N_waypoints = 107 ,pose_x = pose_x_Astar, pose_y = pose_y_Astar)

Lattice = Trajectory()
Lattice.path(planner_name = 'Freespace Lattice',  N_waypoints = 106 , pose_x = pose_x_Lattice, pose_y = pose_y_Lattice)

Op = Trajectory()
Op.path(planner_name = 'Open Planner',  N_waypoints = 192 , pose_x = pose_x_Op, pose_y = pose_y_Op)

Public_Road = Trajectory()
Public_Road.path(planner_name = 'Public_Road ',  N_waypoints = 208 ,pose_x = pose_x_Public_Road, pose_y = pose_y_Public_Road)


f,axes = plt.subplots()
ax = plt.gca()   

ax = Astar.pose_df.plot(kind='line', x='Pose_x', y= Astar.name, linewidth=1.25, color= 'Black', legend=True)     
Lattice.pose_df.plot(ax = ax, kind='line', x='Pose_x', y= Lattice.name, linewidth=1.25, color= 'Red', legend=True)     
Op.pose_df.plot(ax = ax, kind='line', x='Pose_x', y= Op.name, linewidth=1.25, color= 'Green', legend=True)     
Public_Road.pose_df.plot(ax = ax, kind='line', x='Pose_x', y= Public_Road.name, linewidth=1.25, color= 'Blue', legend=True)     

ax.set_xlabel('Longitudinal axis (x)')
ax.set_ylabel('Lateral axis (y)')
ax.legend(loc='upper left', prop={'size':7.7})        

        