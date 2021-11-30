#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 10:23:01 2021

@author: autoware-auto-ros1
"""

from Public_Road_data_to_plot_vel_pose_steer import *
from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_Astar_Mechanic_Energy import *
#from Power_Consumption_Public_Road_Mechanic_Energy import *
from Power_Consumption_OP_Mechanic_Energy import *
import pandas as pd
import time
import datetime as dt
from datetime import datetime
from datetime import timedelta
import math
import numpy as np
from operator import add
import seaborn as sns
from scipy import stats
import matplotlib.pyplot as plt

#OP DATA
Op_vel_xy_list = Op_df_vel
Op_accel_xy_list = Op_accel_xy
#Op_time_list = Op_trip_seconds
Op_steering_list = Op_Steering_yaw_List
Op_Pose = Op_cum_xy_position
#ASTAR DATA
Astar_vel_xy_list = Astar_df_vel
Astar_accel_xy_list = Astar_accel_xy
#Astar_time_list = Astar_trip_seconds
Astar_steering_list = Astar_Steering_yaw_List
Astar_Pose = Astar_cum_xy_position
#LATTICE DATA
Lattice_vel_xy_list = Lattice_df_vel
Lattice_accel_xy_list = Lattice_accel_xy
#Lattice_time_list = Lattice_trip_seconds
Lattice_steering_list = Lattice_Steering_yaw_List
Lattice_Pose = Lattice_cum_xy_position
#PUBLIC_ROAD DATA
Public_Road_vel_xy_list = Public_Road_df_vel['Vel xy']
Public_Road_accel_xy_list = Public_Road_accel_xy
#Public_Road_time_list = Public_Road_trip_seconds
Public_Road_steering_list = Public_Road_Steering_yaw_List
Public_Road_Pose = Public_Road_xy_Position



class Velocity():
    
    
    def correct_imu_noise(self, Steering_List,planner_name):
        
        self.planner_name = planner_name
        self.Steering_List = Steering_List
    #In case of Apollo the IMU is just standardize +90 degrees ahead Autoware...so
        if self.planner_name == 'Public_Road':
            correction_factor = 90
            self.abacaxi = 3
            self.new_heading_apollo_list = []
            for i in range(len(self.Steering_List)):
                deg_old_value = self.Steering_List[i] 
                new_deg_value = deg_old_value - correction_factor
                self.new_heading_apollo_list.append(new_deg_value)
       
#in case of Autoware IMU suddenly change the values from 0 to -360, change the orientation reading 
        else:
            self.degree_list = [math.degrees(x) for x in self.Steering_List]
            correction_factor = 360
            self.new_heading_list = []
            for i in range(len(self.degree_list)):
                deg_old_value = self.degree_list[i]
                if abs(deg_old_value) > 300 and deg_old_value>0:
                    new_deg_value = -correction_factor + deg_old_value
                elif abs(deg_old_value) > 300 and deg_old_value<0:
                    new_deg_value = correction_factor + deg_old_value
                else: 
                    new_deg_value = deg_old_value
                self.new_heading_list.append(new_deg_value)
        
                
            
    
    def plot_vel(self,velocity, acceleration, heading, pose, planner_name):
        
        self.velocity_array = velocity
        self.accel_array = acceleration
        self.pose = pose
        #self.time = time
        self.planner_name = planner_name
        self.heading = heading
        
        
        
        if self.planner_name == 'Public_Road': #The name from fields change for APOLLO Planners
            self.vel_xy_resultant = self.velocity_array #It was already calculated for Apollo
            self.plot_vel_time = pd.DataFrame({self.planner_name :self.vel_xy_resultant,'Pose':self.pose})
            self.plot_accel_time = pd.DataFrame({self.planner_name:self.accel_array,'Pose':self.pose})
            self.plot_heading_time = pd.DataFrame({self.planner_name:self.new_heading_apollo_list,'Pose':self.pose})
        
        else:
            self.velocity_xy = self.velocity_array[['field.twist.linear.x', 'field.twist.linear.y']]
            self.vel_xy_pow = self.velocity_xy.pow(2)
            self.vel_xy_sum = self.vel_xy_pow['field.twist.linear.x'] + self.vel_xy_pow['field.twist.linear.y']
            self.vel_xy_resultant = self.vel_xy_sum**(1/2)
            self.plot_vel_time = pd.DataFrame({self.planner_name :self.vel_xy_resultant,'Pose':self.pose})
            self.plot_accel_time = pd.DataFrame({self.planner_name:self.accel_array,'Pose':self.pose})
            self.plot_heading_time = pd.DataFrame({self.planner_name:self.new_heading_list,'Pose':self.pose})
       
    
       
     
#OBJECTS
#OP       
Op = Velocity()
Op.correct_imu_noise(Steering_List = Op_steering_list, planner_name= 'Open Planner')
Op.plot_vel(velocity = Op_vel_xy_list, acceleration=Op_accel_xy_list, heading = Op_steering_list, pose= Op_Pose, planner_name = 'Open Planner')       
#ASTAR     
Astar = Velocity()
Astar.correct_imu_noise(Steering_List = Astar_steering_list, planner_name='Astar')
Astar.plot_vel(velocity = Astar_vel_xy_list, acceleration=Astar_accel_xy_list, heading = Astar_steering_list, pose= Astar_Pose, planner_name = 'Astar')  
#LATTICE     
Lattice = Velocity()
Lattice.correct_imu_noise(Steering_List = Lattice_steering_list, planner_name='Lattice')
Lattice.plot_vel(velocity = Lattice_vel_xy_list, acceleration=Lattice_accel_xy_list, heading = Lattice_steering_list, pose= Lattice_Pose, planner_name = 'Lattice') 


#PUBLIC_ROAD    
Public_Road = Velocity()
Public_Road.correct_imu_noise(Steering_List = Public_Road_steering_list, planner_name='Public_Road')
Public_Road.plot_vel(velocity = Public_Road_vel_xy_list, acceleration= Public_Road_accel_xy_list, heading = Public_Road_steering_list, pose= Public_Road_Pose, planner_name = 'Public_Road') 


#PLOTS     

f,axes = plt.subplots(3,1,figsize=(22,11))
ax = plt.gca()   


ax = Astar.plot_vel_time.plot(kind='line', x='Pose', linewidth=1, color="Black",legend=True, ax=axes[0])
ax = Astar.plot_accel_time.plot(kind='line',x ='Pose', linewidth=1, color="Black", legend =True, ax=axes[1])
ax = Astar.plot_heading_time.plot(kind='line',x ='Pose', linewidth=1, color="Black", legend =True, ax=axes[2])

Lattice.plot_vel_time.plot(kind='line', x='Pose', linewidth=1, color="Red",legend=True, ax=axes[0])
Lattice.plot_accel_time.plot(kind='line',x ='Pose', linewidth=1, color="Red", legend =True, ax=axes[1])
Lattice.plot_heading_time.plot(kind='line',x ='Pose', linewidth=1, color="Red", legend =True, ax=axes[2])

Op.plot_vel_time.plot(kind='line', x='Pose', linewidth=1, color="Green",legend=True, ax=axes[0])
Op.plot_accel_time.plot(kind='line',x ='Pose', linewidth=1, color="Green", legend =True, ax=axes[1])
Op.plot_heading_time.plot(kind='line',x ='Pose', linewidth=1, color="Green", legend =True, ax=axes[2])


Public_Road.plot_vel_time.plot(kind='line', x='Pose', linewidth=1, color="Blue",legend=True, ax=axes[0])
Public_Road.plot_accel_time.plot(kind='line',x ='Pose', linewidth=1, color="Blue", legend =True, ax=axes[1])
Public_Road.plot_heading_time.plot(kind='line',x ='Pose', linewidth=1, color="Blue", legend =True, ax=axes[2])


axes[0].set_ylabel('Velocity (m/s)',fontsize=13)
axes[0].set_xlabel('')
axes[0].legend(prop={'size':13})
axes[1].set_ylabel('Acceleration (m/s²)',fontsize=13)
axes[1].set_xlabel('')
axes[1].legend(prop={'size':13})
axes[2].set_ylabel('Heading (°) ', fontsize=13)
axes[2].legend(prop={'size':13})



#axes[0].set_title("OPEN PLANNER", color="Black")


axes[2].set_xlabel('Path Position (x)',fontsize=14.25)

        
