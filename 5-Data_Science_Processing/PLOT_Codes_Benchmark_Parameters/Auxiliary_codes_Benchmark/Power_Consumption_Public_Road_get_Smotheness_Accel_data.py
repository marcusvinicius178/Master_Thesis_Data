#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 15:36:25 2021

@author: autoware-auto-ros1
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 15:08:15 2021

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
from scipy.interpolate import InterpolatedUnivariateSpline, UnivariateSpline, CubicSpline

#The only difference from Apollo to Autoware Script is that Apollo get also linear_y velocity data




#Reading from IMU beacasue it has all the needed data to calculate power
#It has not linear velocity but has acceleration, so it is just extract the integral of acceeleration
df_raw_imu = pd.read_csv(r'Public_Road_raw_imu_message_Static_Truck.csv')
df_imu = pd.read_csv(r'Public_Road_corrected_imu_messages_Static_Truck.csv')
# Teh csv data above are not synchronized...however both of data are required
#because the the raw_imu contain linear valocity and accel data, and the imu contain the Orientation yaw angles in Euler...
#Therefore it is needed to merge both on TimeStamp data to drop unmatched TimeStamps
df_velocity = pd.read_csv(r'Public_Road_chassis_Static_Truck.csv')
df_pose = pd.read_csv(r'Public_Road_odometry_messages_Static_Truck.csv')

df_synchronized_imu = df_imu.merge(df_raw_imu, on="TimeStamp", copy=False)
df_synchronized_imu_pose = df_synchronized_imu.merge(df_pose, on="TimeStamp", copy=False)

df_synchronized_imu.to_csv('Merged_imu.csv')
df_synchronized_imu_pose.to_csv('Last.csv')

df_difference = df_synchronized_imu_pose['Position_x'].diff()
df_abs_difference = df_difference.abs()

#Removing the time of modules activation in Autoware.AI Runtime Manager
filtered_real_car_journey = df_synchronized_imu #df_synchronized_imu_pose[df_abs_difference>0.0005]

#remove for single pose Df too to use for calculation of obstacle distance
df_pose_difference = df_pose['Position_x'].diff()
df_abs_pose_diff = df_pose_difference.abs()
filtered_pose = df_pose[df_abs_pose_diff>0.0005]


#df_pose = pd.read_csv(r'current_pose_op.csv')
#df_yaw_angle = pd.read_csv(r'converted_quaternions_to_Euler.csv')

#From manual, still not from LGSVL Unity 3D, just to test
global L
L = float(2.917986)   #float(2.72796) #wheelbase in meters of Lincoln MKZ model from LGSVL UNITY3D
M = int(2120) # Kg taken in LGSVL support - unix measurement, Same for both vehicles

#Check if the csv Counters-Timestamp from files above are aligned, if not, drop the lines manually to synchronize
# and then replace "," to "." because after drop operation it loses the format...and do not work the code..

#Get TimeStamp
time_stamp_array = filtered_real_car_journey['TimeStamp']
#Get LinearAcceleration_X -- After merge the columns name changed from 'LinearAcceleration_x' to 'LinearAcceleration_x_y'
#Must check in the new csv the columns to match exactly
linear_accel_x_array = filtered_real_car_journey['LinearAcceleration_x_y']
#Get LinearAcceleration_Y
linear_accel_y_array = filtered_real_car_journey['LinearAcceleration_y_y']
#Get Angular_velocity_X 
angular_vel_x_array = filtered_real_car_journey['AngularVelocity_x_y']
#Get Angular_velocity_Y 
angular_vel_y_array = filtered_real_car_journey['AngularVelocity_y_y']
#Get Angular_Velocity_Z
angular_vel_z_array = filtered_real_car_journey['AngularVelocity_z_y']
#Get Yaw Angle 
orientation_yaw_z_array = filtered_real_car_journey['EulerAngles_z']
#Get velocity data: Resultant Forward x,y
velocity_forward_xy_array = df_velocity['Speed_mps']
#Get_pose
pose_x_array_Public_Road = filtered_pose['Position_x']
pose_y_array_Public_Road = filtered_pose['Position_y']


#---------------------------------------------------------------------------

#TASKS FOR POWER CALCULATION PSEUDO TUTORIAL

#Delta time Extraction and Conversion to seconds into a Time_List

#TASK 1: CONVERT TIME MACHINE TO HUMAN AND GET VELOCITY VARIATION AND ACCELERATION IN X AXIS



final_list = []
for i in range(len(time_stamp_array)):
    
    time_value = time_stamp_array.iloc[i]
    human_readable = datetime.fromtimestamp(time_value)
    #print(human_readable)
    
    #The code below was required just for Autoware Unix Timestamp...Apollo has better data
    '''
    string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
    
    str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
    #required to drop the last 6 characters related to microseconds otherwise next line will not process it
    str_human_time = str_human_time[:-6]

    human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
    #print(human_time_float)
    '''
    final_list.append(human_readable)





final_list_delta_seconds = []
for t in range(len(final_list)-1):
    
    #date_time_value = final_list[i]
    #print(date_time_value)
    delta_t = final_list[t+1] - final_list[t]
    
    #In miliseconds to seconds conversion
    delta_ms = ( delta_t / dt.timedelta(milliseconds=1) ) *1e-3
    final_list_delta_seconds.append(delta_ms)

    
#-----------------------TRIP TOTAL TIME CALCULATION------------------------------------------------
Trip_seconds_Public_Road = sum(final_list_delta_seconds)
#-------------------------------------------------------------------------------------------------
  
    
    


#DELTA LINEAR ACCELERATION & VELOCITY CALCULATION FROM DELTA ACCELERATION
#For X axis
# ΔV_linear_x =  Δa_linear_accel_x * Δt

final_list_delta_linear_accel_x = []
for a_lin_x in range(len(linear_accel_x_array)-1):
    delta_a_lin_x = linear_accel_x_array.iloc[a_lin_x+1] - linear_accel_x_array.iloc[a_lin_x]
    final_list_delta_linear_accel_x.append(delta_a_lin_x)
    
final_value_average_accel_x = (np.nansum(final_list_delta_linear_accel_x)) / len(final_list_delta_linear_accel_x)
#I did not drop the negative acceleration in Apollo because the arrays x,y would not match afterwards
#to calculate the module...would give a much issue to code match, have not too much time available
#A option would be neglect the y acceleration values as they are too low...anyway the average on x acceleration was pretty similar with negative numbers in contrast with Autoware, that this trick needed to be used...    
    

delta_velocity_x_list = [a * b for a,b in zip(final_list_delta_linear_accel_x, final_list_delta_seconds)]
delta_velocity_x_value_average = np.nansum(delta_velocity_x_list) /len(delta_velocity_x_list)

#The list will contain - and + values, because it depends if the car is braking or throttle.
#However what interest to us is the module of the speed after multiply P = F * V 

#For Y axis
# ΔV_linear_y =  Δa_linear_accel_y * Δt

final_list_delta_linear_accel_y = []
for a_lin_y in range(len(linear_accel_y_array)-1):
    delta_a_lin_y = linear_accel_y_array.iloc[a_lin_y+1] - linear_accel_y_array.iloc[a_lin_y]
    final_list_delta_linear_accel_y.append(delta_a_lin_y)
    #test average
final_value_average_accel_y = (np.nansum(final_list_delta_linear_accel_y)) / len(final_list_delta_linear_accel_y)


delta_velocity_y_list = [c * d for c,d in zip(final_list_delta_linear_accel_y, final_list_delta_seconds)]
delta_velocity_y_value_average = np.nansum(delta_velocity_y_list) /len(delta_velocity_y_list)



#Get the total velocity

linear_vel_pow_x_value = delta_velocity_x_value_average **2
linear_vel_pow_y_value = delta_velocity_y_value_average **2


linear_vel_xy_pow_sum_list = linear_vel_pow_x_value + linear_vel_pow_y_value
linear_velocity_xy_resultant_module_list = math.sqrt(linear_vel_xy_pow_sum_list)

#GET VELOCITY AVERAGE/MEDIAN and NOT THE DELTA VELOCITY (AS CODE ABOVE)
velocity_forward_average = np.nansum(velocity_forward_xy_array) / len(velocity_forward_xy_array)
velocity_forward_median = np.median(velocity_forward_xy_array)
#Both values are pretty similar, no noise, anyone can be used




#------------------------------------------------------------------------------------------------------------------

#TASK2: GET THE ANGULAR VELOCITY VARIATION = Δω = Rad/s
#CCW is positive and CW is negative - However it is not required if condition, as the difference will always show if the move is mainly CCW or CW

final_list_delta_angular_velocity_z = []
for v_z in range(len(angular_vel_z_array)-1):
    angular_velocity = angular_vel_z_array.iloc[i]
    delta_v_z = angular_vel_z_array.iloc[v_z+1] - angular_vel_z_array.iloc[v_z]
    absolute_diff = delta_v_z  # To calculate the power does not matter the direction of acceleration, just the module how much the car is taking of energy 
    final_list_delta_angular_velocity_z.append(absolute_diff)
    
#------------------------------------------------------------------------------------------------------------------    

#TASK3: Get the Rate of Angular Acceleration = θ = Δω / Δt 
acceleration_z_list = [i /j for i,j in zip(final_list_delta_angular_velocity_z, final_list_delta_seconds)]


#------------------------------------------------------------------------------------------------------------------

#TASK4: Calculate the Total Acceleration = Anet = sqrt (atencial² + acentripetal²)
# A) Tangential Acceleration


Steering_yaw_list = []
for steer in range(len(orientation_yaw_z_array)-1):
#At first the vehicle is turning CCW to avoid obstacle, therefore the Delta value is + (convetion)
#Then it returns to the Lane, turning CW and the new yaw/steer values are negative. So a if condition must be done        
#In contrast with AUTOWARE data, here in APollo the yaw angle starts at 90 deg as reference so the IMU never get negative values
#For this reason the code will be simple with if condition    

    steer_yaw_value = orientation_yaw_z_array.iloc[steer]
 #   if steer_yaw_value < 0: # CCW = + yaw
    delta_steer_value = steer_yaw_value - orientation_yaw_z_array.iloc[0] #because the first value is starting with negative value (-3 = -178 deg) and next is (-2 turning CCW) = the difference is 1 rad, so -2 - (-3) = -2 +3 = 1 rad
    absolute_delta_steer = delta_steer_value #The same here as angular velocity, does not matter if the car rotated CCW or CW to calculate the power...
 #   else:
        #delta_steer_value = steer_yaw_value + orientation_yaw_array[0]
        #absolute_delta_steer = abs(delta_steer_value)#The module will make easy later to calculate the ICR (positived always) and Tangential Acceleration
    Steering_yaw_list.append(absolute_delta_steer)


#calculate below the tangent(Steer = alpha =yaw) array 
tan_yaw_list = []


for tan_a in range(len(Steering_yaw_list)):
    tan_yaw_value = math.tan(Steering_yaw_list[tan_a])
    tan_yaw_list.append(tan_yaw_value)



#Calculate the Instantaneous Center of Rotation (ICR)
# R = ICR = tan(alpha) * L , where L is the vehicle's wheelbase
ICR_List = []
for rad in range(len(tan_yaw_list)):
    icr_value = L * tan_yaw_list[rad]
    ICR_List.append(icr_value)


#Finally the tangential acceleration is given by 
#at = θ x R
Tangential_Acceleration_List =  [k * l for k,l in zip(acceleration_z_list, ICR_List)]

#-----------------------------------------------------------------------------------------------------------------

#Task4 
# B) Calculating the Centripetal Acceleration --> ac = r*ω² , where r = ICR and w = angular velocity
#w²    
Power_delta_angular_vel_z = [f ** 2 for f in final_list_delta_angular_velocity_z]
#ac
accel_centripetal_list = [g * h for g,h in zip(ICR_List, Power_delta_angular_vel_z)]

#-----------------------------------------------------------------------------------------------------------------

#TASK 5 Calculation of Total Acceleration
#TOTAL ACCELERATION = Anet = sqrt (acentripetal² + at²)

Acentripetal_Power_List = [i ** 2 for i in accel_centripetal_list]
Atangential_Power_List = [j ** 2 for j in Tangential_Acceleration_List]

Sum_accel_List = list(map(add, Acentripetal_Power_List, Atangential_Power_List)) 
#Sum_accel_List = [sum(k) for k in zip(Acentripetal_Power_List, Atangential_Power_List)]

Anet_List = np.sqrt(Sum_accel_List)

#----------------------------------------------------------------------------------------------------
#Intermediate Calculation: THETA ANGLE
#Calculate the theta angle between the net force and x|y axis, to further calculation of components of NET acceleration

Beta_arctan_List = []

for tan_beta in range(len(Sum_accel_List)):
    beta_angle_value = math.atan(Acentripetal_Power_List[tan_beta] / Atangential_Power_List[tan_beta])
    Beta_arctan_List.append(beta_angle_value)


omega_intermediate_angle_List = []

for omega in range(len(Beta_arctan_List)):
    omega_value = Beta_arctan_List[omega] + Steering_yaw_list[omega]
    omega_intermediate_angle_List.append(omega_value)
    
Theta_decomposition_angle_List = []

for theta in range(len(omega_intermediate_angle_List)):
    Theta_value = 1.5708 - omega_intermediate_angle_List[theta]
    Theta_decomposition_angle_List.append(theta)
                   
                   
#Finally Decomposing the resultant of rotational acceleration in x and y components

acceleration_x_from_rotational_net_List = []

for accel_x_rot in range(len(Theta_decomposition_angle_List)):
    accel_x_component_value = Anet_List[accel_x_rot] * (math.sin(Theta_decomposition_angle_List[accel_x_rot]))
    acceleration_x_from_rotational_net_List.append(accel_x_component_value)

drop_negative_accel_values_x_list =  np.array( [num for num in acceleration_x_from_rotational_net_List if num>=0])
acceleration_x_from_rot_anet_average = np.nansum(drop_negative_accel_values_x_list) / len(drop_negative_accel_values_x_list)


acceleration_y_from_rotational_net_List = []

for accel_y_rot in range(len(Theta_decomposition_angle_List)):
    accel_y_component_value = Anet_List[accel_y_rot] * (math.cos(Theta_decomposition_angle_List[accel_y_rot]))
    acceleration_y_from_rotational_net_List.append(accel_y_component_value)


drop_negative_accel_values_y_list =  np.array( [num for num in acceleration_y_from_rotational_net_List if num>=0])
acceleration_y_from_rot_anet_average = np.nansum(drop_negative_accel_values_y_list) / len(drop_negative_accel_values_y_list)


#--------------copy this code to the old one module.py--------------------
Total_Accel_x = list(map(add, final_list_delta_linear_accel_x, acceleration_x_from_rotational_net_List))
Total_Accel_y = list(map(add, final_list_delta_linear_accel_y, acceleration_y_from_rotational_net_List))

Accel_x_component_Power_List = [v ** 2 for v in Total_Accel_x]
Accel_y_component_Power_List = [w ** 2 for w in Total_Accel_y]
#--------------------------------------------------------------------------------------

Sum_accel_components_List = list(map(add, Accel_x_component_Power_List, Accel_y_component_Power_List)) 
Accel_xy_resultant_components = np.sqrt(Sum_accel_components_List)




#-------------------------------------------------------------------------------------------------
#TASK 7 
#Join the Acceleration x, acceleration y and delta pose list, in order to separate the positive values (Power Hp from accel) and negative values (brake)
#Joined_accel_pose_df = pd.DataFrame({'Acceleration XY':linear_acceleration_xy_resultant_module_list,
 #                                    'Delta Pose XY': delta_pose_xy_module_list,
  #                                   'Delta Time': final_list_delta_seconds})

Join_acceleration_df = pd.DataFrame({'Acceleration X':Total_Accel_x,
                                     'Acceleration Y': Total_Accel_y,
                                     'Delta Time': final_list_delta_seconds})


# REMOVING THE OUTLIERS: Before keeping the calculation of Force and Power it is requiered
#Identifyng the Outliers to remove
sns.boxplot(Join_acceleration_df["Acceleration X"])



#Using IQR 
Q1 = Join_acceleration_df[["Acceleration X", "Acceleration Y"]].quantile(0.25)
Q3 = Join_acceleration_df[["Acceleration X", "Acceleration Y"]].quantile(0.75)
IQR = Q3 - Q1
#print(IQR)
Outliers_fst_quartile = Join_acceleration_df[["Acceleration X", "Acceleration Y"]] < (Q1 - 1.5 * IQR) 
Outliers_4th_quartile = Join_acceleration_df[["Acceleration X", "Acceleration Y"]] > (Q3 + 1.5 * IQR)

remove_Outliers_df_IQR_method = Join_acceleration_df[~((Join_acceleration_df < (Q1 - 1.5 * IQR)) |(Join_acceleration_df > (Q3 + 1.5 * IQR))).any(axis=1)]

Filtered_Parameters_Df = remove_Outliers_df_IQR_method


Acceleration_List = Filtered_Parameters_Df #[(Filtered_Parameters_Df[['Acceleration X']] > 0).all(1)]

Acceleration_xy_df = Acceleration_List[['Acceleration X', 'Acceleration Y']]


Acceleration_xy_Pow_df = Acceleration_xy_df.pow(2)
Sum_Acceleration_xy_pow_df = Acceleration_xy_Pow_df['Acceleration X'] + Acceleration_xy_Pow_df['Acceleration Y']
Resultant_Acceleration_xy_Sqrt_List = Sum_Acceleration_xy_pow_df**(1/2)
Resultant_accel_df = pd.DataFrame({'Acceleration XY':Resultant_Acceleration_xy_Sqrt_List})
Acceleration_rejoin_delta_df = pd.merge(Acceleration_List['Delta Time'], Resultant_accel_df['Acceleration XY'], left_index=True, right_index=True)
Acceleration_xy_df_Public_Road = Acceleration_rejoin_delta_df

