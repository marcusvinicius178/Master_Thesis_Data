#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  8 11:53:50 2021

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

df_pose_orientation = pd.read_csv(r'Op_current_pose_for_Deviation_calc.csv')
df_pose_orientation_final_wp = pd.read_csv(r'Op_final_waypoints_yaw.csv')

'''
df_synchronized_pose_orientation = df_pose_orientation_final_wp.merge(df_pose_orientation, on="%time", copy=False)
df_synchronized_pose_orientation.to_csv('Pose_Orientation_Merged.csv')
'''

time_stamp_array_final_wp = df_pose_orientation_final_wp["%time"]
time_stamp_array_current_pose = df_pose_orientation["%time"]

final_list_final_wp = []
for i in range(len(time_stamp_array_final_wp)):
    
    time_value = time_stamp_array_final_wp.iloc[i]
    human_readable = datetime.fromtimestamp(time_value // 1e9)
    string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
    
    str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
    #required to drop the last 6 characters related to microseconds otherwise next line will not process it
    str_human_time = str_human_time[:-6]

    human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
    #print(human_time_float)
    final_list_final_wp.append(human_time_float)


final_list_current_pose = []
for i in range(len(time_stamp_array_current_pose)):
    
    time_value = time_stamp_array_current_pose.iloc[i]
    human_readable = datetime.fromtimestamp(time_value // 1e9)
    string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
    
    str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
    #required to drop the last 6 characters related to microseconds otherwise next line will not process it
    str_human_time = str_human_time[:-6]

    human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
    #print(human_time_float)
    final_list_current_pose.append(human_time_float)



Redo_Df_final_waypoints = pd.DataFrame({'TimeStamp':final_list_final_wp})
append_df_final_wp = df_pose_orientation_final_wp.join(Redo_Df_final_waypoints)

Redo_Df_current_pose = pd.DataFrame({'TimeStamp':final_list_current_pose})
append_df = df_pose_orientation.join(Redo_Df_current_pose)


#df_pose_orientation_final_wp['%time'] = df_pose_orientation_final_wp['%time'].map(Redo_Df_final_waypoints.set_index('%time')[Redo_Df_final_waypoints])

df_synchronized_pose_orientation = append_df_final_wp.merge(append_df, on="TimeStamp", copy=False)
df_synchronized_pose_orientation.to_csv('Visualize_Merged_Df.csv')
#-------------------------EXTRACT ARRAYS x,y,z for each DATAFRAME SOURCE-----------------------


Deviation_x_List = []
Deviation_y_List = []
Deviation_z_List = []

for dev in range(len(df_synchronized_pose_orientation)-1):
    x_deviation = df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x'].iloc[dev] - df_synchronized_pose_orientation['field.pose.position.x'].iloc[dev]
    y_deviation = df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.y'].iloc[dev] - df_synchronized_pose_orientation['field.pose.position.y'].iloc[dev]
    #To do the calculation for z it is required to convert quaternion to Euler(yaw) from final waypoints.....as it was done during rosbag extraction for current_pose
    z_deviation = df_synchronized_pose_orientation['field.waypoints0.pose.pose.orientation.z.yaw'].iloc[dev] - df_synchronized_pose_orientation['yaw'].iloc[dev]
    
    Deviation_x_List.append(x_deviation)
    Deviation_y_List.append(y_deviation)
    Deviation_z_List.append(z_deviation)
    
    
Deviation_Df = pd.DataFrame({'x_dev':Deviation_x_List, 'y_dev':Deviation_y_List, 'yaw_dev':Deviation_z_List})    
Deviation_xy_Df = Deviation_Df[['x_dev','y_dev']]    
Deviation_xy_pow_Df = Deviation_xy_Df.pow(2)
Sum_Deviation_xy_Df = Deviation_xy_pow_Df['x_dev'] + Deviation_xy_pow_Df['y_dev']
Resultant_xy_Deviation_Df = Sum_Deviation_xy_Df**(1/2)


pose_x_list = df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x']
#get relative position in x    
relative_position_x = []
        #self.abs_relative_pose_x = []
for rel_x in range(len(pose_x_list)-1):
    rel_value = pose_x_list[rel_x] - pose_x_list[0]
    relative_position_x.append(rel_value)
    

relative_position_x = [-i for i in relative_position_x]

plot_xy_Deviation_Df = pd.DataFrame({'Path Position':relative_position_x,
                                     'Deviation Pose':Resultant_xy_Deviation_Df})
    
df_difference = plot_xy_Deviation_Df['Path Position'].diff()
df_abs_difference = df_difference.abs()
filter_redundant_origin_pose_x = plot_xy_Deviation_Df[df_abs_difference>0.01]
origin_df_recover_path = [plot_xy_Deviation_Df.iloc[0]['Path Position']]
origin_df_recover_deviation_xy = [plot_xy_Deviation_Df.iloc[0]['Deviation Pose']]

origin_df_recover_Df = pd.DataFrame({'Path Position':origin_df_recover_path,
                                     'Deviation Pose':origin_df_recover_deviation_xy})

New_Df_recovered_origin = origin_df_recover_Df.append(filter_redundant_origin_pose_x)


New_Df_recovered_origin.plot('Path Position', 'Deviation Pose')

