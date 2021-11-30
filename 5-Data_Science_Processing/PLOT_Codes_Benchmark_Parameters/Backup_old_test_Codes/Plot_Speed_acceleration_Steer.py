#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 10:23:01 2021

@author: autoware-auto-ros1
"""

from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_Astar_Mechanic_Energy import *
from Power_Consumption_Public_Road_Mechanic_Energy import *
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

Op_vel_xy_list = Op_df_vel
Op_accel_xy_list = Op_accel_xy
Op_time_list = Op_trip_seconds
Op_steering_list = Op_Steering_yaw_List


class Velocity():
    
    def plot_vel(self,velocity, acceleration, heading, time, planner_name):
        
        self.velocity_array = velocity
        self.accel_array = acceleration
        self.time = time
        self.planner_name = planner_name
        self.heading = heading
        
        self.velocity_xy = self.velocity_array[['field.twist.linear.x', 'field.twist.linear.y']]
        self.vel_xy_pow = self.velocity_xy.pow(2)
        self.vel_xy_sum = self.vel_xy_pow['field.twist.linear.x'] + self.vel_xy_pow['field.twist.linear.y']
        self.vel_xy_resultant = self.vel_xy_sum**(1/2)
        
        self.plot_vel_time = pd.DataFrame({'Velocity' :self.vel_xy_resultant,'Time':self.time})
        self.plot_accel_time = pd.DataFrame({'Acceleration':self.accel_array,'Time':self.time})
        self.plot_heading_time = pd.DataFrame({'Heading':self.heading, 'Time':self.time})
       
       
#OBJECTS
#OP       
Op = Velocity()
Op.plot_vel(velocity = Op_vel_xy_list, acceleration=Op_accel_xy_list, heading = Op_steering_list, time=Op_time_list, planner_name = 'Open Planner')       
#ASTAR




#PLOTS     

f,axes = plt.subplots(3,1,figsize=(10,6))
ax = plt.gca()   

ax = Op.plot_vel_time.plot(kind='line', x='Time', linewidth=1, color="Green",legend=True, ax=axes[0])
ax = Op.plot_accel_time.plot(kind='line',x ='Time', linewidth=1, color="Green", legend =True, ax=axes[1])
ax = Op.plot_heading_time.plot(kind='line',x ='Time', linewidth=1, color="Green", legend =True, ax=axes[2])

axes[0].set_ylabel('Velocity (m/s)')
axes[1].set_ylabel('Acceleration (m/s²)')
axes[2].set_ylabel('Heading (rad/s) ')
axes[0].set_title("OPEN PLANNER", color="Black")


ax.set_xlabel('Time Trip (s)')

        
