#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 22:21:46 2021

@author: autoware-auto-ros1
"""
import pandas as pd
import math
from tf.transformations import euler_from_quaternion

df_orientation_final_wp = pd.read_csv(r'Op_final_waypoints.csv')


orientation_x = df_orientation_final_wp['field.waypoints0.pose.pose.orientation.x']
orientation_y = df_orientation_final_wp['field.waypoints0.pose.pose.orientation.y']
orientation_z = df_orientation_final_wp['field.waypoints0.pose.pose.orientation.z']
orientation_w = df_orientation_final_wp['field.waypoints0.pose.pose.orientation.w']

#orientation_df = pd.DataFrame({'x':orientation_x,'y':orientation_y,'z':orientation_z,'w':orientation_w})

yaw_list = []
for x,y,z,w in zip(orientation_x,orientation_y,orientation_z,orientation_w):
    orientation_list = [x,y,z,w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    yaw_list.append(yaw)
    
    
degree_list = []
for degree in range(len(yaw_list)):
    value_deg = math.degrees(yaw_list[degree])
    degree_list.append(value_deg)
    
    

    
    
Orientation_Df = pd.DataFrame({'field.waypoints0.pose.pose.orientation.z.yaw':yaw_list,
                                  'yaw_in_degrees':degree_list})

#appending the new columns to original Dataframe

dat1 = df_orientation_final_wp.reset_index(drop=True)
dat2 = Orientation_Df.reset_index(drop=True)

Op_Orientation_yaw_final_wp = pd.concat([dat1,dat2],axis=1)
Op_Orientation_yaw_final_wp.to_csv('Op_final_waypoints_yaw.csv')

