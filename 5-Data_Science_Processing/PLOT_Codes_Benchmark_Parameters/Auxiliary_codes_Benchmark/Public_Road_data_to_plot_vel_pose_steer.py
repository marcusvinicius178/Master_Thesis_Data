#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 15:07:46 2021

@author: autoware-auto-ros1
"""
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
from scipy import stats
from sympy import *
import sympy as sp
from statistics import stdev, variance
import regex as re
from functools import reduce


df_velocity = pd.read_csv(r'Public_Road_chassis_Static_Truck.csv')
df_pose = pd.read_csv(r'Public_Road_odometry_messages_Static_Truck.csv')
df_acceleration = pd.read_csv((r'Public_Road_raw_imu_message_Static_Truck.csv'))


pose_x_array_Public_Road = df_pose['Position_x']
pose_y_array_Public_Road = df_pose['Position_y']


time_stamp_array_velocity = df_pose['TimeStamp']
time_stamp_array_pose = df_pose['TimeStamp']
time_stamp_array_acceleration = df_acceleration['TimeStamp']

time_vel_list = []

for i in range(len(time_stamp_array_velocity)):
            
    time_value = time_stamp_array_velocity.iloc[i]
    human_readable = datetime.fromtimestamp(time_value)
    s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
    #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
    new_s = s[:-4]
    
    time_vel_list.append(new_s)


time_pose_list = []

for i in range(len(time_stamp_array_pose)):
            
    time_value = time_stamp_array_pose.iloc[i]
    human_readable = datetime.fromtimestamp(time_value)
    s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
    #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
    new_s = s[:-4]
    
    time_pose_list.append(new_s)
    

time_accel_list = []

for i in range(len(time_stamp_array_acceleration)):
            
    time_value = time_stamp_array_acceleration.iloc[i]
    human_readable = datetime.fromtimestamp(time_value)
    s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
    #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
    new_s = s[:-4]
    
    time_accel_list.append(new_s)


#DATAFRAME of velocity DATA  
Redo_velocity_df = pd.DataFrame({'TimeSynchronize':time_vel_list})
append_vel_df = df_velocity.join(Redo_velocity_df)
#DATAFRAME of pose DATA
Redo_pose_df = pd.DataFrame({'TimeSynchronize':time_pose_list})
append_pose_df = df_pose.join(Redo_pose_df)


#Required to synchronize these DataFrames above on TimeSTamp column in order to subtract the corresponding waypoints on time        


#data_frames = [append_vel_df, append_pose_df, append_acceleration_df]
#df_synchronized_vel_pose_accel = reduce(lambda  left,right: pd.merge(left,right,on=['TimeSynchronize'],
                                            #how='outer'), data_frames)

df_synchronized_vel_pose = pd.merge(append_pose_df, append_vel_df, on="TimeSynchronize")
df_synchronized_vel_pose.to_csv('Visualize_Merged_Df_Apollo_new.csv')



time_df_synchronized_vel_pose = df_synchronized_vel_pose['TimeStamp_x']
sync_vel_pose_list = []

for i in range(len(time_df_synchronized_vel_pose)):
    time_value = time_df_synchronized_vel_pose.iloc[i]
    human_readable = datetime.fromtimestamp(time_value)
    s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
    #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
    
    #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
    new_s = s[:-4]
    
    sync_vel_pose_list.append(new_s)


#DATAFRAME of pose_velocity DATA  
Redo_pose_velocity_df = pd.DataFrame({'New_TimeSynchronize':sync_vel_pose_list})
append_pose_vel_df = df_synchronized_vel_pose.join(Redo_pose_velocity_df)
#DATAFRAME of acceleration data (imu)
Redo_acceleration_df = pd.DataFrame({'New_TimeSynchronize':time_accel_list})
append_acceleration_df = df_acceleration.join(Redo_acceleration_df)

df_synchronized_vel_pose_accel = append_pose_vel_df.merge(append_acceleration_df, on="New_TimeSynchronize", copy=False)
df_synchronized_vel_pose_accel.to_csv('Visualize_Merged_Df_Apollo_new.csv')



#AFTER THIS HARD TIMESTAMP SYNCHRONIZATION, FINALLY GETTING THE DATA
vel_xy_df = df_synchronized_vel_pose_accel['Speed_mps']
linear_accel_x_array = df_synchronized_vel_pose_accel['LinearAcceleration_x']
linear_accel_y_array = df_synchronized_vel_pose_accel['LinearAcceleration_y']
pose_x_df = df_synchronized_vel_pose_accel['Position_x']
orientation_yaw_array = df_synchronized_vel_pose_accel['Heading']
Public_Road_Steering_yaw_List = orientation_yaw_array



#GET VELOCITY XY TO PLOT
Public_Road_df_vel = pd.DataFrame({'Vel xy': vel_xy_df })


#GET ACCELERATION XY YO PLOT
Accel_xy_Df = pd.DataFrame({'Acceleration x':linear_accel_x_array, 'Acceleration y':linear_accel_y_array})
Accel_xy_df_pow = Accel_xy_Df.pow(2)
Accel_xy_df_pow_sum = Accel_xy_df_pow['Acceleration x'] + Accel_xy_df_pow['Acceleration y']
Accel_xy_df_resultant = Accel_xy_df_pow_sum**(1/2)
Public_Road_accel_xy = Accel_xy_df_resultant
#GET Relative X and CumX


final_pose = df_pose['Position_x'].iloc[-1]
origin_pose = df_pose['Position_x'].iloc[0]
Total_path = abs(final_pose-origin_pose)

    
Public_Road_xy_Pose = np.linspace(0,Total_path,len(Public_Road_df_vel))
Public_Road_xy_Position = list(Public_Road_xy_Pose)




