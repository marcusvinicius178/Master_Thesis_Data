#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 22:21:46 2021

@author: autoware-auto-ros1
"""
import pandas as pd
import math
from tf.transformations import euler_from_quaternion

Op_df_orientation_final_wp = pd.read_csv(r'Op_final_waypoints.csv')

Astar_df_orientation_final_wp = pd.read_csv(r'Astar_final_waypoints.csv')
#It was required to cut some excessive number of columns in Lattice_final_waypoints.csv, because pandas was unable to read so many points produced
#It was needed also to substitue all commas ',' values to '.' using ctrl+H func to substitute
Lattice_df_orientation_final_wp = pd.read_csv(r'Lattice_final_waypoints_cut_excess_columnsmanually.csv')



class Conv_quat_to_Euler():
    def __init__(self, df_orientation_final_wp):

        self.df_orientation_final_wp = df_orientation_final_wp

        self.orientation_x = self.df_orientation_final_wp['field.waypoints0.pose.pose.orientation.x']
        self.orientation_y = self.df_orientation_final_wp['field.waypoints0.pose.pose.orientation.y']
        self.orientation_z = self.df_orientation_final_wp['field.waypoints0.pose.pose.orientation.z']
        self.orientation_w = self.df_orientation_final_wp['field.waypoints0.pose.pose.orientation.w']
        
        #orientation_df = pd.DataFrame({'x':orientation_x,'y':orientation_y,'z':orientation_z,'w':orientation_w})
    def conversion_function(self):
        
        self.yaw_list = []
        for x,y,z,w in zip(self.orientation_x,self.orientation_y,self.orientation_z,self.orientation_w):
            self.orientation_list = [x,y,z,w]
            (self.roll,self.pitch,self.yaw) = euler_from_quaternion(self.orientation_list)
            self.yaw_list.append(self.yaw)
            
            
        self.degree_list = []
        for degree in range(len(self.yaw_list)):
            value_deg = math.degrees(self.yaw_list[degree])
            self.degree_list.append(value_deg)
            
            
        
            
            
        self.Orientation_Df = pd.DataFrame({'field.waypoints0.pose.pose.orientation.z.yaw':self.yaw_list,
                                          'yaw_in_degrees':self.degree_list})
        
        #appending the new columns to original Dataframe
        
        self.dat1 = self.df_orientation_final_wp.reset_index(drop=True)
        self.dat2 = self.Orientation_Df.reset_index(drop=True)
        
        self.Orientation_yaw_final_wp = pd.concat([self.dat1,self.dat2],axis=1)
       
        
Op = Conv_quat_to_Euler(df_orientation_final_wp= Op_df_orientation_final_wp)
Op.conversion_function()
Op.Orientation_yaw_final_wp.to_csv('Op_final_waypoints_yaw.csv')

Astar = Conv_quat_to_Euler(df_orientation_final_wp= Astar_df_orientation_final_wp)
Astar.conversion_function()
Astar.Orientation_yaw_final_wp.to_csv('Astar_final_waypoints_yaw.csv')

Lattice = Conv_quat_to_Euler(df_orientation_final_wp= Lattice_df_orientation_final_wp)
Lattice.conversion_function()
Lattice.Orientation_yaw_final_wp.to_csv('Lattice_final_waypoints_yaw.csv')
