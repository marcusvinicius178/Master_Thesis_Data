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

#OP DATA
Op_df_pose_orientation = pd.read_csv(r'Op_current_pose_for_Deviation_calc.csv')
Op_df_pose_orientation_final_wp = pd.read_csv(r'Op_final_waypoints_yaw.csv')
Op_time_stamp_array_final_wp = Op_df_pose_orientation_final_wp["%time"]
Op_time_stamp_array_current_pose = Op_df_pose_orientation["%time"]
#ASTAR DATA
Astar_df_pose_orientation = pd.read_csv(r'Astar_current_pose_for_Deviation_calc.csv')
Astar_df_pose_orientation_final_wp = pd.read_csv(r'Astar_final_waypoints_yaw.csv')
Astar_time_stamp_array_final_wp = Astar_df_pose_orientation_final_wp["%time"]
Astar_time_stamp_array_current_pose = Astar_df_pose_orientation["%time"]
#LATTICE DATA
Lattice_df_pose_orientation = pd.read_csv(r'Lattice_current_pose_for_Deviation_calc.csv')
Lattice_calculate_waypoints = pd.read_csv(r'lattice_calculate_waypoints.csv', error_bad_lines=False)
Lattice_df_pose_orientation_final_wp = pd.read_csv(r'Lattice_final_waypoints_yaw.csv')
Lattice_time_stamp_array_final_wp = Lattice_df_pose_orientation_final_wp["%time"]
Lattice_time_stamp_array_current_pose = Lattice_df_pose_orientation["%time"]
#PUBLIC ROAD (APOLLO PLANNER) DATA 
#Public_Road_Df_Deviation = pd.read_csv(r'Public_Road_control_Static_Truck.csv')
Public_Road_df_pose_orientation = pd.read_csv(r'Public_Road_localization_Static_Truck.csv')
Public_Road_pose_orientation_final_wp = pd.read_csv(r'Public_Road_planning_Static_Truck.csv') 

Public_Road_time_stamp_array_final_wp = Public_Road_pose_orientation_final_wp['TimeStamp']
Public_Road_time_stamp_array_current_pose = Public_Road_df_pose_orientation['TimeStamp']

#Apollo_lateral_deviation_array = Public_Road_Df_Deviation['Lateral_Error']
#Apollo_heading_deviation_array = Public_Road_Df_Deviation['Heading_Error']


#Not possible to synchronize on Timestamp, Apollo used different timestamps for different cyber channels. Solution is get a numpy.linspace for X PAth..it is a good approximation
#much better than create the code to round Timestamp which seems to be complex, considering must be done Unix Timestamp conversion
#Public_Road_Synchronize_Df = Public_Road_Df_Deviation.merge(Public_Road_Df_pose, on="TimeStamp", copy=False)
#Public_Road_Synchronize_Df_2 = Public_Road_Df_pose.merge(Public_Road_Df_pose, on="TimeStamp", copy=False)




class Deviation_xyz():
    

    def deviation_lists_autoware(self, df_pose_orientation, df_pose_orientation_final_wp,time_stamp_array_final_wp, time_stamp_array_current_pose, planner_name):

        self.df_pose_orientation = df_pose_orientation
        self.df_pose_orientation_final_wp = df_pose_orientation_final_wp
        self.time_stamp_array_final_wp = time_stamp_array_final_wp
        self.time_stamp_array_current_pose = time_stamp_array_current_pose
        
        self.planner_name = planner_name


        self.final_list_final_wp = []
        for i in range(len(self.time_stamp_array_final_wp)):
            
            time_value = time_stamp_array_final_wp.iloc[i]
            human_readable = datetime.fromtimestamp(time_value // 1e9)
            string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
            
            str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
            #required to drop the last 6 characters related to microseconds otherwise next line will not process it
            str_human_time = str_human_time[:-6]
        
            human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
            #print(human_time_float)
            s = human_time_float.strftime('%Y-%m-%d %H:%M:%S.%f')
            #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
            new_s = s[:-5]
            
            self.final_list_final_wp.append(new_s)
        
        
        self.final_list_current_pose = []
        for i in range(len(self.time_stamp_array_current_pose)):
            
            time_value = time_stamp_array_current_pose.iloc[i]
            human_readable = datetime.fromtimestamp(time_value // 1e9)
            string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
            
            str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
            #required to drop the last 6 characters related to microseconds otherwise next line will not process it
            str_human_time = str_human_time[:-6]
        
            human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
            #print(human_time_float)
            s = human_time_float.strftime('%Y-%m-%d %H:%M:%S.%f')
            #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
            new_s = s[:-5]
            
            self.final_list_current_pose.append(new_s)
        
        
      #DATAFRAME of final_waypoints topic Data  
        self.Redo_Df_final_waypoints = pd.DataFrame({'TimeStamp':self.final_list_final_wp})
        self.append_df_final_wp = self.df_pose_orientation_final_wp.join(self.Redo_Df_final_waypoints)
    #DATAFRAME of current_pose topic DATA
        self.Redo_Df_current_pose = pd.DataFrame({'TimeStamp':self.final_list_current_pose})
        self.append_df = self.df_pose_orientation.join(self.Redo_Df_current_pose)
    #Required to synchronize these DataFrames above on TimeSTamp column in order to subtract the corresponding waypoints on time        
        
        
        self.df_synchronized_pose_orientation = self.append_df_final_wp.merge(self.append_df, on="TimeStamp", copy=False)
        self.df_synchronized_pose_orientation.to_csv('Visualize_Merged_Df.csv')
        #-------------------------EXTRACT ARRAYS x,y,z for each DATAFRAME SOURCE-----------------------
        
        
        self.Deviation_x_List = []
        self.Deviation_y_List = []
        self.Deviation_z_List = []
        
        for dev in range(len(self.df_synchronized_pose_orientation)-1):
            #x_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x'].iloc[dev] - self.df_synchronized_pose_orientation['field.pose.position.x'].iloc[dev]
            #y_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.y'].iloc[dev] - self.df_synchronized_pose_orientation['field.pose.position.y'].iloc[dev]
            x_deviation = self.df_synchronized_pose_orientation['field.pose.position.x'].iloc[dev] - self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x'].iloc[dev]
            y_deviation = self.df_synchronized_pose_orientation['field.pose.position.y'].iloc[dev] - self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.y'].iloc[dev]
            #To do the calculation for z it is required to convert quaternion to Euler(yaw) from final waypoints.....as it was done during rosbag extraction for current_pose
            #z_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.orientation.z.yaw'].iloc[dev] - self.df_synchronized_pose_orientation['yaw'].iloc[dev]
            z_deviation =   abs(self.df_synchronized_pose_orientation['yaw'].iloc[dev]) - abs(self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.orientation.z.yaw'].iloc[dev])

            
            self.Deviation_x_List.append(x_deviation)
            self.Deviation_y_List.append(y_deviation)
            self.Deviation_z_List.append(z_deviation)
            
            
        self.Deviation_Df = pd.DataFrame({'x_dev':self.Deviation_x_List, 'y_dev':self.Deviation_y_List, 'yaw_dev':self.Deviation_z_List})    
        self.Deviation_xy_Df = self.Deviation_Df[['x_dev','y_dev']]    
        self.Deviation_xy_pow_Df = self.Deviation_xy_Df.pow(2)
        self.Sum_Deviation_xy_Df = self.Deviation_xy_pow_Df['x_dev'] + self.Deviation_xy_pow_Df['y_dev']
        self.Resultant_xy_Deviation_Df = self.Sum_Deviation_xy_Df**(1/2)
        
        
        self.pose_x_list = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x']
        #get relative position in x    
        self.relative_position_x = []
                #self.abs_relative_pose_x = []
        for rel_x in range(len(self.pose_x_list)-1):
            rel_value = self.pose_x_list[rel_x] - self.pose_x_list[0]
            self.relative_position_x.append(rel_value)
            
        
        self.relative_position_x = [-i for i in self.relative_position_x]
        
        self.plot_xy_Deviation_Df = pd.DataFrame({'Path Position':self.relative_position_x,
                                             'Deviation Pose':self.Resultant_xy_Deviation_Df})
            
        self.df_difference = self.plot_xy_Deviation_Df['Path Position'].diff()
        self.df_abs_difference = self.df_difference.abs()
        self.filter_redundant_origin_pose_x = self.plot_xy_Deviation_Df[self.df_abs_difference>0.01]
    
#Add a standar last position to DataFrame
        self.last_pose_to_df = pd.DataFrame({'Path Position':[80], 'Deviation Pose':[None]})
        self.add_standart_last_pose = self.filter_redundant_origin_pose_x.append(self.last_pose_to_df) 
        self.add_standart_last_pose_sort = self.add_standart_last_pose.sort_values(by=['Path Position'])
        self.add_standart_last_pose_sort.reset_index(drop=True, inplace=True)
        self.add_standart_last_pose_sort_interpolate = self.add_standart_last_pose_sort.interpolate()

#Remove the additional Path
        self.filter_additional_path_x = self.add_standart_last_pose_sort_interpolate[self.add_standart_last_pose_sort_interpolate['Path Position']<80.5]


        self.origin_df_recover_path = [self.plot_xy_Deviation_Df.iloc[0]['Path Position']]
        self.origin_df_recover_deviation_xy = [self.plot_xy_Deviation_Df.iloc[0]['Deviation Pose']]
        
        self.origin_df_recover_Df = pd.DataFrame({'Path Position':self.origin_df_recover_path,
                                             'Deviation Pose':self.origin_df_recover_deviation_xy})
        
        self.xy_deviation_autoware = self.origin_df_recover_Df.append(self.filter_additional_path_x)
        self.xy_deviation_autoware_column_rename = self.xy_deviation_autoware.rename(columns={'Deviation Pose': self.planner_name})

        
        #self.xy_deviation_autoware.plot('Path Position', 'Deviation Pose')
        
# DEVIATION YAW ANGLE = HEADING

    def deviation_lists_autoware_heading(self, df_pose_orientation, df_pose_orientation_final_wp,time_stamp_array_final_wp, time_stamp_array_current_pose, planner_name):
        self.df_pose_orientation = df_pose_orientation
        self.df_pose_orientation_final_wp = df_pose_orientation_final_wp
        self.time_stamp_array_final_wp = time_stamp_array_final_wp
        self.time_stamp_array_current_pose = time_stamp_array_current_pose
        
        self.planner_name = planner_name


        self.final_list_final_wp = []
        for i in range(len(self.time_stamp_array_final_wp)):
            
            time_value = time_stamp_array_final_wp.iloc[i]
            human_readable = datetime.fromtimestamp(time_value // 1e9)
            string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
            
            str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
            #required to drop the last 6 characters related to microseconds otherwise next line will not process it
            str_human_time = str_human_time[:-6]
        
            human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
            #print(human_time_float)
            s = human_time_float.strftime('%Y-%m-%d %H:%M:%S.%f')
            #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
            new_s = s[:-6]
            
            self.final_list_final_wp.append(new_s)
        
        
        self.final_list_current_pose = []
        for i in range(len(self.time_stamp_array_current_pose)):
            
            time_value = time_stamp_array_current_pose.iloc[i]
            human_readable = datetime.fromtimestamp(time_value // 1e9)
            string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
            
            str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
            #required to drop the last 6 characters related to microseconds otherwise next line will not process it
            str_human_time = str_human_time[:-6]
        
            human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
            #print(human_time_float)
            s = human_time_float.strftime('%Y-%m-%d %H:%M:%S.%f')
            #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
            new_s = s[:-6]
            
            self.final_list_current_pose.append(new_s)
        
        
      #DATAFRAME of final_waypoints topic Data  
        self.Redo_Df_final_waypoints = pd.DataFrame({'TimeStamp':self.final_list_final_wp})
        self.append_df_final_wp = self.df_pose_orientation_final_wp.join(self.Redo_Df_final_waypoints)
    #DATAFRAME of current_pose topic DATA
        self.Redo_Df_current_pose = pd.DataFrame({'TimeStamp':self.final_list_current_pose})
        self.append_df = self.df_pose_orientation.join(self.Redo_Df_current_pose)
    #Required to synchronize these DataFrames above on TimeSTamp column in order to subtract the corresponding waypoints on time        
        
        
        self.df_synchronized_pose_orientation = self.append_df_final_wp.merge(self.append_df, on="TimeStamp", copy=False)
        self.df_synchronized_pose_orientation.to_csv('Visualize_Merged_Df.csv')
        #-------------------------EXTRACT ARRAYS x,y,z for each DATAFRAME SOURCE-----------------------
        
        
        self.Deviation_x_List = []
        self.Deviation_y_List = []
        self.Deviation_z_List = []
        
        for dev in range(len(self.df_synchronized_pose_orientation)-1):
            #x_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x'].iloc[dev] - self.df_synchronized_pose_orientation['field.pose.position.x'].iloc[dev]
            #y_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.y'].iloc[dev] - self.df_synchronized_pose_orientation['field.pose.position.y'].iloc[dev]
            x_deviation = self.df_synchronized_pose_orientation['field.pose.position.x'].iloc[dev] - self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.x'].iloc[dev]
            y_deviation = self.df_synchronized_pose_orientation['field.pose.position.y'].iloc[dev] - self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.position.y'].iloc[dev]
            #To do the calculation for z it is required to convert quaternion to Euler(yaw) from final waypoints.....as it was done during rosbag extraction for current_pose
            #z_deviation = self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.orientation.z.yaw'].iloc[dev] - self.df_synchronized_pose_orientation['yaw'].iloc[dev]
            z_deviation =   abs(self.df_synchronized_pose_orientation['yaw'].iloc[dev]) - abs(self.df_synchronized_pose_orientation['field.waypoints0.pose.pose.orientation.z.yaw'].iloc[dev])

            
            self.Deviation_x_List.append(x_deviation)
            self.Deviation_y_List.append(y_deviation)
            self.Deviation_z_List.append(z_deviation)
      
        
        
                    
        self.Deviation_Df = pd.DataFrame({'x_dev':self.Deviation_x_List, 'y_dev':self.Deviation_y_List, 'yaw_dev':self.Deviation_z_List})    
        #DIFFERENCE ON FUNCTIONS START BELOW

        self.Deviation_heading = self.Deviation_Df['yaw_dev']     
        self.Deviation_heading_x_path = pd.DataFrame({'Path Position':self.relative_position_x,
                                                      'Heading Deviation':self.Deviation_heading})
        self.filter_redundant_origin_pose_x_yaw = self.Deviation_heading_x_path[self.df_abs_difference>0.01]
        #Add standard last position do Yaw Df
        self.last_pose_to_yaw_df = pd.DataFrame({'Path Position':[80], 'Heading Deviation':[None]})
        self.add_standart_last_pose_yaw = self.filter_redundant_origin_pose_x_yaw.append(self.last_pose_to_yaw_df) 
        self.add_standart_last_pose_yaw_sort = self.add_standart_last_pose_yaw.sort_values(by=['Path Position'])
        self.add_standart_last_pose_yaw_sort.reset_index(drop=True, inplace=True)
        self.add_standart_last_pose_yaw_sort_interpolate = self.add_standart_last_pose_yaw_sort.interpolate()
        #Removing the additional Path >80
        self.filter_additional_path_x_yaw = self.add_standart_last_pose_yaw_sort_interpolate[self.add_standart_last_pose_yaw_sort_interpolate['Path Position']<80.5]
        #Recovering origin DF
        self.origin_df_recover_path_x_yaw = [self.Deviation_heading_x_path.iloc[0]['Path Position']]
        self.origin_df_recover_deviation_yaw = [self.Deviation_heading_x_path.iloc[0]['Heading Deviation']]
        
        self.origin_yaw_recover_Df = pd.DataFrame({'Path Position':self.origin_df_recover_path_x_yaw,
                                             'Heading Deviation':self.origin_df_recover_deviation_yaw})
        
        self.heading_deviation_autoware = self.origin_yaw_recover_Df.append(self.filter_additional_path_x_yaw)
        self.heading_deviation_autoware_column_rename = self.heading_deviation_autoware.rename(columns={'Heading Deviation': self.planner_name})
        
        #self.heading_deviation_autoware.plot('Path Position', 'Heading Deviation')
        
      
      
class Deviation_xyz_Apollo(): 

    def deviation_lists_apollo(self, df_pose_orientation, df_pose_orientation_final_wp,time_stamp_array_final_wp, time_stamp_array_current_pose, planner_name):
        
        self.planner_name = planner_name
        self.df_pose_orientation = df_pose_orientation
        self.df_pose_orientation_final_wp = df_pose_orientation_final_wp
        self.time_stamp_array_final_wp = time_stamp_array_final_wp
        self.time_stamp_array_current_pose = time_stamp_array_current_pose
        
        
        
        self.final_list_final_wp = []
        #self.ts_pandas = []
        for i in range(len(self.time_stamp_array_final_wp)):
            
            time_value = self.time_stamp_array_final_wp.iloc[i]
            human_readable = datetime.fromtimestamp(time_value)
            s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
            #FILTER MILISECONDS SPLITTIN LAST MICLISECONDS IN ORDER TO BETTER SUNCHRONIZATION
            new_s = s[:-6] #get just first milisecond value
            #ts = pd.Timestamp(human_readable)
            #ts.round(freq='S')
           
            #self.ts_pandas.append(ts) 
            self.final_list_final_wp.append(new_s)
        
        self.final_list_current_pose = []
        for i in range(len(self.time_stamp_array_current_pose)):
            
            time_value = self.time_stamp_array_current_pose.iloc[i]
            human_readable = datetime.fromtimestamp(time_value)
            s = human_readable.strftime('%Y-%m-%d %H:%M:%S.%f')
            new_s = s[:-6]
           
            self.final_list_current_pose.append(new_s)
     
    #DATAFRAME of final_waypoints topic Data  
        self.Redo_Df_final_waypoints = pd.DataFrame({'TimeStamp_Apollo':self.final_list_final_wp})
        self.append_df_final_wp = self.df_pose_orientation_final_wp.join(self.Redo_Df_final_waypoints)
    #DATAFRAME of current_pose topic DATA
        self.Redo_Df_current_pose = pd.DataFrame({'TimeStamp_Apollo':self.final_list_current_pose})
        self.append_df = self.df_pose_orientation.join(self.Redo_Df_current_pose)
    #Required to synchronize these DataFrames above on TimeSTamp column in order to subtract the corresponding waypoints on time        
        
        
        self.df_synchronized_pose_orientation = self.append_df_final_wp.merge(self.append_df, on="TimeStamp_Apollo", copy=False)
        self.df_synchronized_pose_orientation_only = self.df_synchronized_pose_orientation[["Position_x_y","Position_y_y","Heading_y","Position_x_x","Position_y_x","Heading_x"]]
        self.df_synchronized_pose_orientation_only.to_csv('Visualize_Merged_Df_Apollo.csv')
        self.df_synchronized_pose_orientation_only_rem_nan = self.df_synchronized_pose_orientation_only.dropna()


        self.Deviation_x_List = []
        self.Deviation_y_List = []
        self.Deviation_z_List = []
        
        for dev in range(len(self.df_synchronized_pose_orientation_only_rem_nan)-1):
            #Deviation is the Dataframe of current_pose from AV - planned_pose_waypoint from planner data
            x_deviation = self.df_synchronized_pose_orientation_only_rem_nan['Position_x_y'].iloc[dev] - self.df_synchronized_pose_orientation_only_rem_nan['Position_x_x'].iloc[dev]
            y_deviation = self.df_synchronized_pose_orientation_only_rem_nan['Position_y_y'].iloc[dev] - self.df_synchronized_pose_orientation_only_rem_nan['Position_y_x'].iloc[dev]
            z_deviation =  abs(self.df_synchronized_pose_orientation_only_rem_nan['Heading_y'].iloc[dev]) - abs(self.df_synchronized_pose_orientation_only_rem_nan['Heading_x'].iloc[dev])

            
            self.Deviation_x_List.append(x_deviation)
            self.Deviation_y_List.append(y_deviation)
            self.Deviation_z_List.append(z_deviation)
            
        self.Deviation_Df = pd.DataFrame({'x_dev':self.Deviation_x_List, 'y_dev':self.Deviation_y_List, 'yaw_dev':self.Deviation_z_List})    
        self.Deviation_xy_Df = self.Deviation_Df[['x_dev','y_dev']]    
        self.Deviation_xy_pow_Df = self.Deviation_xy_Df.pow(2)
        self.Sum_Deviation_xy_Df = self.Deviation_xy_pow_Df['x_dev'] + self.Deviation_xy_pow_Df['y_dev']
        self.Resultant_xy_Deviation_Df = self.Sum_Deviation_xy_Df**(1/2)
        #Droppiong NaN values from List 
        #self.Resultant_xy_Deviation_Df =  [x for x in self.Resultant_xy_Deviation_Df if math.isnan(x) == False] 


        self.pose_x_list = self.df_synchronized_pose_orientation_only_rem_nan['Position_x_x']
        #get relative position in x    
        self.relative_position_x = []
                #self.abs_relative_pose_x = []
        for rel_x in range(len(self.pose_x_list)-1):
            rel_value = self.pose_x_list.iloc[rel_x] - self.pose_x_list.iloc[0]
            self.relative_position_x.append(rel_value)
            
        self.relative_position_x = [-i for i in self.relative_position_x]
        
        self.plot_xy_Deviation_Df = pd.DataFrame({'Path Position':self.relative_position_x,
                                             'Deviation Pose':self.Resultant_xy_Deviation_Df})
            
        self.df_difference = self.plot_xy_Deviation_Df['Path Position'].diff()
        self.df_abs_difference = self.df_difference.abs()
        self.filter_redundant_origin_pose_x = self.plot_xy_Deviation_Df[self.df_abs_difference>0.01]
    
#Add a standar last position to DataFrame
        self.last_pose_to_df = pd.DataFrame({'Path Position':[80], 'Deviation Pose':[None]})
        self.add_standart_last_pose = self.filter_redundant_origin_pose_x.append(self.last_pose_to_df) 
        self.add_standart_last_pose_sort = self.add_standart_last_pose.sort_values(by=['Path Position'])
        self.add_standart_last_pose_sort.reset_index(drop=True, inplace=True)
        self.add_standart_last_pose_sort_interpolate = self.add_standart_last_pose_sort.interpolate()

#Remove the additional Path
        self.filter_additional_path_x = self.add_standart_last_pose_sort_interpolate[self.add_standart_last_pose_sort_interpolate['Path Position']<80.5]


        self.origin_df_recover_path = [self.plot_xy_Deviation_Df.iloc[0]['Path Position']]
        self.origin_df_recover_deviation_xy = [self.plot_xy_Deviation_Df.iloc[0]['Deviation Pose']]
        
        self.origin_df_recover_Df = pd.DataFrame({'Path Position':self.origin_df_recover_path,
                                             'Deviation Pose':self.origin_df_recover_deviation_xy})
        
        self.xy_deviation_apollo = self.origin_df_recover_Df.append(self.filter_additional_path_x)
        self.xy_deviation_apollo_column_rename = self.xy_deviation_apollo.rename(columns={'Deviation Pose': self.planner_name})

# DEVIATION YAW ANGLE = HEADING
   
        self.Deviation_heading = self.Deviation_Df['yaw_dev']     
        self.Deviation_heading_x_path = pd.DataFrame({'Path Position':self.relative_position_x,
                                                      'Heading Deviation':self.Deviation_heading})
        self.filter_redundant_origin_pose_x_yaw = self.Deviation_heading_x_path[self.df_abs_difference>0.01]
        #Add standard last position do Yaw Df
        self.last_pose_to_yaw_df = pd.DataFrame({'Path Position':[80], 'Heading Deviation':[None]})
        self.add_standart_last_pose_yaw = self.filter_redundant_origin_pose_x_yaw.append(self.last_pose_to_yaw_df) 
        self.add_standart_last_pose_yaw_sort = self.add_standart_last_pose_yaw.sort_values(by=['Path Position'])
        self.add_standart_last_pose_yaw_sort.reset_index(drop=True, inplace=True)
        self.add_standart_last_pose_yaw_sort_interpolate = self.add_standart_last_pose_yaw_sort.interpolate()
        #Removing the additional Path >80
        self.filter_additional_path_x_yaw = self.add_standart_last_pose_yaw_sort_interpolate[self.add_standart_last_pose_yaw_sort_interpolate['Path Position']<80.5]
        #Recovering origin DF
        self.origin_df_recover_path_x_yaw = [self.Deviation_heading_x_path.iloc[0]['Path Position']]
        self.origin_df_recover_deviation_yaw = [self.Deviation_heading_x_path.iloc[0]['Heading Deviation']]
        
        self.origin_yaw_recover_Df = pd.DataFrame({'Path Position':self.origin_df_recover_path_x_yaw,
                                             'Heading Deviation':self.origin_df_recover_deviation_yaw})
        
        self.heading_deviation_apollo = self.origin_yaw_recover_Df.append(self.filter_additional_path_x_yaw)
        self.heading_deviation_apollo_column_rename = self.heading_deviation_apollo.rename(columns={'Heading Deviation': self.planner_name})
        




#Open Planner
Op = Deviation_xyz()
Op.deviation_lists_autoware(df_pose_orientation=Op_df_pose_orientation, df_pose_orientation_final_wp=Op_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Op_time_stamp_array_final_wp, time_stamp_array_current_pose=Op_time_stamp_array_current_pose, planner_name='Open Planner')
Op.deviation_lists_autoware_heading(df_pose_orientation=Op_df_pose_orientation, df_pose_orientation_final_wp=Op_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Op_time_stamp_array_final_wp, time_stamp_array_current_pose=Op_time_stamp_array_current_pose, planner_name='Open Planner')

#ASTAR 
Astar = Deviation_xyz()
Astar.deviation_lists_autoware(df_pose_orientation=Astar_df_pose_orientation, df_pose_orientation_final_wp=Astar_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Astar_time_stamp_array_final_wp, time_stamp_array_current_pose=Astar_time_stamp_array_current_pose, planner_name='Astar')
Astar.deviation_lists_autoware_heading(df_pose_orientation=Astar_df_pose_orientation, df_pose_orientation_final_wp=Astar_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Astar_time_stamp_array_final_wp, time_stamp_array_current_pose=Astar_time_stamp_array_current_pose, planner_name='Astar')


#Lattice
Lattice = Deviation_xyz()
Lattice.deviation_lists_autoware(df_pose_orientation=Lattice_df_pose_orientation, df_pose_orientation_final_wp=Lattice_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Lattice_time_stamp_array_final_wp, time_stamp_array_current_pose=Lattice_time_stamp_array_current_pose, planner_name='Lattice')
Lattice.deviation_lists_autoware_heading(df_pose_orientation=Lattice_df_pose_orientation, df_pose_orientation_final_wp=Lattice_df_pose_orientation_final_wp,
                   time_stamp_array_final_wp=Lattice_time_stamp_array_final_wp, time_stamp_array_current_pose=Lattice_time_stamp_array_current_pose, planner_name='Lattice')


#WARNING!!!: THE PUBLIC ROAD PLANNING DATA COULD NOT BE EXTRACTED DUE SOME ISSUE ON THIS CHANNEL
#COMMUNICATION WITH THE SIMULATOR, THE POSITION X AND POSITION Y RETURNED NAN VALUES...
#FOR THIS REASON IT IS NOT POSSIBLE TO CALCULATE THE DEVIATION POSE XY = PLANNING(X) - ODOM(X)
#HOWEVER APOLLO RETURNS THE LATERAL ERROR IN OTHER CSV, LATERAL_ERROR = LINEAR_VEL * SIN(DELTA_THETA = YAW)



#Public Road
Public_Road = Deviation_xyz_Apollo( )
Public_Road.deviation_lists_apollo(df_pose_orientation = Public_Road_df_pose_orientation, df_pose_orientation_final_wp = Public_Road_pose_orientation_final_wp, time_stamp_array_final_wp = Public_Road_time_stamp_array_final_wp, time_stamp_array_current_pose = Public_Road_time_stamp_array_current_pose, planner_name = 'Public Road')



#-------------------------------PLOT DATA CODE BELOW -------------------------------------

f, axes = plt.subplots(2,1, figsize=(9,4))


ax = plt.gca()




#Open Planner
ax = Op.xy_deviation_autoware_column_rename.plot(kind='line', x='Path Position', y= Op.planner_name, linewidth=1, color= 'Green', ax=axes[0], legend= False)
Astar.xy_deviation_autoware_column_rename.plot(ax= ax, kind='line', x='Path Position', linewidth=1.25, color= 'Black', legend=False)
Lattice.xy_deviation_autoware_column_rename.plot(ax= ax, kind='line', x='Path Position', linewidth=1, color= 'Red', legend=False)
Public_Road.xy_deviation_apollo_column_rename.plot(ax = ax, kind='line', x='Path Position', linewidth=1, color='Blue', legend=False)
ax.set_ylabel('Distance to Path')


ax = Op.heading_deviation_autoware_column_rename.plot(kind='line', x='Path Position', y= Op.planner_name, linewidth=1, color= 'Green', legend=True, ax=axes[1])
Astar.heading_deviation_autoware_column_rename.plot(ax= ax, kind='line', x='Path Position', linewidth=2, color= 'Black', legend=True)
Lattice.heading_deviation_autoware_column_rename.plot(ax= ax, kind='line', x='Path Position', linewidth=1, color= 'Red', legend=True)
Public_Road.heading_deviation_apollo_column_rename.plot(ax = ax, kind='line', x='Path Position', linewidth =1, color= 'Blue', legend=True)

ax.set_ylabel('Heading Deviation')

ax.legend(loc='lower left', prop={'size':6})

#IT IS POSSIBLE TO CALCULATE FOR APOLLO TOO. I HAVE NOOT TOOK ZOOM OUT TO SEE THE DATA
#COMING AFTER LINE 56

