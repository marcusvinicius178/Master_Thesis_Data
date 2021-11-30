#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  6 12:00:45 2021

@author: autoware-auto-ros1
"""

from Power_Consumption_Astar_Mechanic_Energy import *
from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_Public_Road_get_Smotheness_Accel_data import * #different module that returns more data from x,y position
from Power_Consumption_OP_Mechanic_Energy import *
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from scipy import stats
import pandas as pd
import regex as re

#Extract the position of obstacle using REgex match pattern
Velodyne_Perception_Obstacle_Df = pd.read_csv(r'Public_Road_velodyne_lidar_Static_Truck.csv')
Obstacle_df = Velodyne_Perception_Obstacle_Df['Perception_Obstacles_Matrix']
Obstacle_regex = Velodyne_Perception_Obstacle_Df
#match = Obstacle_regex[Obstacle_regex['Perception_Obstacles_Matrix'].str.match('^polygon_point.*')== True]
S = pd.Series(Obstacle_regex['Perception_Obstacles_Matrix'])
Obstacle_rear_part = S.iloc[0] #The rear is the first part detected from velodyne
extraction_obstacle_rear_pose_x = (re.search('x: (\d+)', Obstacle_rear_part)).group(1)
extraction_obstacle_rear_pose_y = (re.search('y: (\d+)', Obstacle_rear_part)).group(1)
obstacle_rear_x_to_float = float(extraction_obstacle_rear_pose_x)
obstacle_rear_y_to_float = float(extraction_obstacle_rear_pose_y)




class Distance_to_Obstacle():
    def __init__(self, obstacle_rear_x_pose, obstacle_rear_y_pose):
        #convert coordinates to relative coordinates
        self.obstacle_rear_x_origin_pose = abs(obstacle_rear_x_pose - pose_x_array_Public_Road.iloc[0])
        self.obstacle_rear_y_origin_pose = abs(obstacle_rear_y_pose - pose_y_array_Public_Road.iloc[0])

    

    def relative_position_planner(self, pose_x_array, pose_y_array, planner_name):
     
       
        self.relative_position_x = []
        self.abs_relative_pose_x = []
        for rel_x in range(len(pose_x_array)-1):
            rel_value = pose_x_array.iloc[rel_x] - pose_x_array.iloc[0]
            self.abs_relative_pose_x.append(rel_value)
            self.relative_position_x.append(rel_value)
        self.abs_relative_pose_x = [-i for i in self.abs_relative_pose_x]    
        self.relative_position_x = [-i for i in self.relative_position_x]    

    
    
        self.relative_position_y = []
        for rel_y in range(len(pose_y_array)-1):
            rel_value = pose_y_array.iloc[rel_y] - pose_y_array.iloc[0]
            self.relative_position_y.append(rel_value)
        
        self.relative_position_y = [-i for i in self.relative_position_y]
    
        
    #Get the approach distance for each waypoint of trajectory: Distance of AV to obstacle
    #in X axis 
    
        self.relative_distance_from_obstacle_x = []
        for rel_dist_x in range(len(self.relative_position_x)):
            dist_obst_value_x = self.relative_position_x[rel_dist_x] - self.obstacle_rear_x_origin_pose
            self.relative_distance_from_obstacle_x.append(dist_obst_value_x)
        
        self.min_val, self.idx = min([(abs(val), idx) for (idx, val) in enumerate(self.relative_distance_from_obstacle_x)])
        if self.min_val < float(0.0):
            self.new_idx = self.idx+1
        elif self.min_val > float(0.0):
            self.new_idx = self.idx

        self.relative_distance_from_obstacle_x.insert(self.new_idx, float(0.0))


        self.absolute_distance_from_obstacle_x = [abs(ele_x) for ele_x in self.relative_distance_from_obstacle_x]
        #Inserting distance = 0. Ussually not available as the posex-y was filtered due TimeStamp Synchronization and does not have all values
        #self.min_index_x = self.absolute_distance_from_obstacle_x.index(min(self.absolute_distance_from_obstacle_x))
        #self.absolute_distance_from_obstacle_x.insert(self.min_index_x+1,0)
        
        #Get lateral distance from obstacle
        self.relative_distance_from_obstacle_y = []
        for rel_dist_y in range(len(self.relative_position_y)):
            dist_obst_value_y = self.relative_position_y[rel_dist_y] - self.obstacle_rear_y_origin_pose
            self.relative_distance_from_obstacle_y.append(dist_obst_value_y)
            
        self.absolute_distance_from_obstacle_y = [abs(ele_y) for ele_y in self.relative_distance_from_obstacle_y]    
        self.aux_np_array = np.array(self.absolute_distance_from_obstacle_y)
        self.absolute_distance_from_obstacle_y = np.insert(self.aux_np_array, self.new_idx, np.nan, axis=0)
       
        self.aux_x_np_array = np.array(self.abs_relative_pose_x)
        self.abs_relative_pose_x_aux = np.insert(self.aux_x_np_array, self.new_idx,np.nan, axis=0 )
    
    
    
        #Get maximun and minimum distance in x of AV to obstacle
        self.Max_x_distance = max(self.absolute_distance_from_obstacle_x)
        self.Min_x_distance = min(self.absolute_distance_from_obstacle_x)
        #Get maximun and minimum distance in y of AV to obstacle
        self.Max_y_distance = max(self.absolute_distance_from_obstacle_y)
        self.Min_y_distance = min(self.absolute_distance_from_obstacle_y)
        
        
        self.distance_to_obstacle_xy =  pd.DataFrame({'Longitudinal Distance':self.absolute_distance_from_obstacle_x,
                                                      'Lateral Distance':self.absolute_distance_from_obstacle_y})
        self.aux_abs_path_x = pd.DataFrame({'Longitudinal Distance':self.absolute_distance_from_obstacle_x,
                                            'Path x':self.abs_relative_pose_x_aux})
        
        self.Interpolate_y_dist_nan = self.distance_to_obstacle_xy.interpolate()
        self.abs_relative_pose_x_plot = self.aux_abs_path_x.interpolate()
        self.abs_relative_pose_x_plot = self.abs_relative_pose_x_plot['Path x']
        
        
        self.distance_to_obstacle_x = pd.DataFrame({'Longitudinal Distance':self.absolute_distance_from_obstacle_x})
        self.distance_to_obstacle_y = pd.DataFrame({'Lateral Distance':self.absolute_distance_from_obstacle_y})
        
        
    
    #sns.lineplot(data=Public_Road_distance_to_obstacle_x['Longitudinal Distance'])
    #sns.lineplot(data=Public_Road_distance_to_obstacle_y['Lateral Distance'])
    
    #THE DISTANCE BELOW IS WHEN X=0, IN OTHER WORDS, WHEN THE CAR IS PARALLELL =EQUAL TO OBSTACLE POSITION IN X
        '''
        ax = plt.gca()
        
        Delta_y_str = str(round(planner_name_Interpolate_y_dist_nan['Lateral Distance'][min_index_x+1],3))
        planner_name_Interpolate_y_dist_nan.plot(kind='line', y='Lateral Distance', use_index=True)
        plt.axvline(x=min_index_x+1, color='Black', linestyle='--')
        plt.text(min_index_x+1,2.5,'ΔY ='+ Delta_y_str,rotation=-12.5)
        plt.show()
        #-------------------------------------------------------------------------------------------
        '''
        
        #CALCULATING THE MINIMUM DISTANCE FROM OBSTACLE sqrt(XY²)
        self.xy_obstacle_distance_pow_df = self.Interpolate_y_dist_nan.pow(2)
        self.Sum_distance_obstacle_xy_pow = self.xy_obstacle_distance_pow_df['Longitudinal Distance'] + self.xy_obstacle_distance_pow_df['Lateral Distance']
        self.xy_distance_obstacle_df = self.Sum_distance_obstacle_xy_pow**(1/2)
        self.xy_dist_obst_List = self.xy_distance_obstacle_df.to_list()
        self.min_xy_val, self.xy_idx = min([(abs(val), idx) for (idx, val) in enumerate(self.xy_dist_obst_List)])

        #self.xy_dist_obst_List_index = [(idx, item) for idx,item in enumerate(self.xy_dist_obst_List)]
        self.xy_dist_obst_index_df = pd.DataFrame({'Distance XY':self.xy_dist_obst_List})
        
        #self.min_index_xy = self.xy_dist_obst_index_df.idxmin()
        self.xy_obst_dist_Df = pd.DataFrame({'Total Obstacle Distance (xy)':self.xy_dist_obst_List, 
                                             'Path Position (x)':self.abs_relative_pose_x_plot}) #self.relative_distance_from_obstacle_x})
        self.min_index_xy = self.xy_obst_dist_Df['Total Obstacle Distance (xy)'].idxmin()

        #self.add_relative_position_obs_dist_Df = pd.        
        


        self.ax = plt.gca()
        self.Delta_xy_str = str(round(self.xy_obst_dist_Df['Total Obstacle Distance (xy)'][self.min_index_xy],3))
        self.xy_obst_dist_Df.plot(kind='line', x='Path Position (x)', y='Total Obstacle Distance (xy)')
        plt.axvline(x= self.xy_obst_dist_Df['Path Position (x)'][self.xy_idx], color='Black', linestyle='--')
        self.text = str('ΔXYmin = '+str(self.Delta_xy_str))
        #plt.text(0.1,0.9,'ΔYmin ='+ self.Delta_xy_str,rotation=0)
        #plt.show()
        left, width = .0, 1
        bottom, height = .0, 1
        right = left + width
        top = bottom + height
        p = plt.Rectangle((left, bottom), width, height, fill=False)
        p.set_transform(self.ax.transAxes)
        p.set_clip_on(False)
        self.ax.add_patch(p)
        
        self.ax.text(0.3 * (left + right), 0.05 * (bottom + top), self.text,
        horizontalalignment='center',
        verticalalignment='center',
        transform=self.ax.transAxes)


#JUST ADDED THE REPEATED PLOT BELOW BECAUSE FOR SOME REASON  1 OF THE PLOT PUT LEGEND WRONG IN EMPTY IMAGE
Scratch = Distance_to_Obstacle(obstacle_rear_x_pose = obstacle_rear_x_to_float , obstacle_rear_y_pose = obstacle_rear_y_to_float )
Scratch.relative_position_planner(pose_x_array = pose_x_array_Lattice, pose_y_array = pose_y_array_Lattice, planner_name = 'Freespace_Lattice')
#--------------------------------------------------------------------------------------------


Public_Road = Distance_to_Obstacle(obstacle_rear_x_pose = obstacle_rear_x_to_float , obstacle_rear_y_pose = obstacle_rear_y_to_float )
Public_Road.relative_position_planner(pose_x_array = pose_x_array_Public_Road, pose_y_array = pose_y_array_Public_Road, planner_name = 'Public_Road')

Op = Distance_to_Obstacle(obstacle_rear_x_pose = obstacle_rear_x_to_float , obstacle_rear_y_pose = obstacle_rear_y_to_float )
Op.relative_position_planner(pose_x_array = pose_x_array_Op, pose_y_array = pose_y_array_Op, planner_name = 'Open_Planner')

Astar = Distance_to_Obstacle(obstacle_rear_x_pose = obstacle_rear_x_to_float , obstacle_rear_y_pose = obstacle_rear_y_to_float )
Astar.relative_position_planner(pose_x_array = pose_x_array_Astar, pose_y_array = pose_y_array_Astar, planner_name = 'Freespace_Astar')

Lattice = Distance_to_Obstacle(obstacle_rear_x_pose = obstacle_rear_x_to_float , obstacle_rear_y_pose = obstacle_rear_y_to_float )
Lattice.relative_position_planner(pose_x_array = pose_x_array_Lattice, pose_y_array = pose_y_array_Lattice, planner_name = 'Freespace_Lattice')
