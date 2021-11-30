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
filtered_real_car_journey = df_synchronized_imu_pose[df_abs_difference>0.0005]



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
pose_x_array = filtered_real_car_journey['Position_x']
pose_y_array = filtered_real_car_journey['Position_y']
#Send to plot-obstacle-distance.py
pose_x_array_Public_Road = pose_x_array
pose_y_array_Public_Road = pose_y_array


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

#GET THE MODULE OF THE ACCELERATION in X and Y that the motor  is realizing
# At = sqrt(a_lin_x² + a_lin_y²)
#Calculation a²

'''
Join_acceleration_df = pd.DataFrame({'Acceleration X':final_list_delta_linear_accel_x,
                                     'Acceleration Y': final_list_delta_linear_accel_y,
                                     'Delta Pose XY': delta_pose_xy_module_list,
                                     'Delta Time': final_list_delta_seconds})

Positive_acceleration_df = Join_acceleration_df[(Join_acceleration_df[['Acceleration X']] > 0).all(1)]
Brake_Acceleration_df = Join_acceleration_df[(Join_acceleration_df[['Acceleration X']] < 0).all(1)]

Positive_Acceleration_x_list = Positive_acceleration_df['Acceleration X'].to_list()
Acceleration_y_list = Positive_acceleration_df['Acceleration Y'].to_list()


linear_accel_pow_x_list = [e **2 for e in Positive_Acceleration_x_list]
linear_accel_pow_y_list = [f **2 for f in Acceleration_y_list]


linear_acceleration_xy_pow_sum_list = [g + h for g,h in zip(linear_accel_pow_x_list, linear_accel_pow_y_list)]
linear_acceleration_xy_resultant_module_list = np.sqrt(linear_acceleration_xy_pow_sum_list)
linear_accel_xy_average = np.nansum (linear_acceleration_xy_resultant_module_list) / len(linear_acceleration_xy_resultant_module_list)
#test calculation of average acceleration module
'''

#GET VELOCITY XY TO PLOT
Velocity_xy_Df = pd.DataFrame({'Vel x':angular_vel_x_array, 'Vel y':angular_vel_x_array})
Vel_xy_df_pow = Velocity_xy_Df.pow(2)
Vel_xy_df_pow_sum = Vel_xy_df_pow['Vel x'] + Vel_xy_df_pow['Vel y']
Vel_xy_df_resultant = Vel_xy_df_pow_sum**(1/2)
Public_Road_df_vel = Vel_xy_df_resultant
#GET ACCELERATION XY YO PLOT
Accel_xy_Df = pd.DataFrame({'Acceleration x':linear_accel_x_array, 'Acceleration y':linear_accel_y_array})
Accel_xy_df_pow = Accel_xy_Df.pow(2)
Accel_xy_df_pow_sum = Accel_xy_df_pow['Acceleration x'] + Accel_xy_df_pow['Acceleration y']
Accel_xy_df_resultant = Accel_xy_df_pow_sum**(1/2)
Public_Road_accel_xy = Accel_xy_df_resultant





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

Public_Road_Steering_yaw_List = Steering_yaw_list
Public_Road_Steering_yaw_List.insert(0,0)


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
#----------------------------------------------------------------------------------------


Accel_x_component_Power_List = [v ** 2 for v in acceleration_x_from_rotational_net_List]
Accel_y_component_Power_List = [w ** 2 for w in acceleration_y_from_rotational_net_List]
Sum_accel_components_List = list(map(add, Accel_x_component_Power_List, Accel_y_component_Power_List)) 

#Test
Total_xy_sum = list(map(add,Total_Accel_x, Total_Accel_y))
Accel_xy_resultant_components = np.sqrt(Total_xy_sum) #np.sqrt(Sum_accel_components_List)


#Accel_xy_resultant_components_average = (np.nansum(Accel_xy_resultant_components)) / len(Accel_xy_resultant_components)
#Accel_xy_resultant_components_median = np.median(Accel_xy_resultant_components)


Accel_xy_resultant_components_average = np.sqrt(acceleration_x_from_rot_anet_average**2 + acceleration_y_from_rot_anet_average**2)


#-----------------------------------------------------------------------------------
#Intermediate Task 6: Get the Delta Pose

final_list_delta_pose_x = []
for p_x in range(len(pose_x_array)-1):
    
    delta_pose_x_value = abs(pose_x_array.iloc[p_x+1]) - abs(pose_x_array.iloc[p_x])
    final_list_delta_pose_x.append(delta_pose_x_value)

delta_x_pose_abs_list = [abs(element) for element in final_list_delta_pose_x]
Sum_delta_x_pose_abs_list_Public_Road = sum(delta_x_pose_abs_list)

Public_Road_cum_xy_position = np.cumsum(delta_x_pose_abs_list)
Public_Road_xy_position = np.insert(Public_Road_cum_xy_position,0,0)


length = len(delta_x_pose_abs_list)
Cummulative_delta_x_pose_list = [sum(delta_x_pose_abs_list[0:x:1]) for x in range(0, length+1)]
Cummulative_delta_x_pose_list.pop()

#To calculate total displacement in Y, the sum of Delta negatives hide the true navigated displacement
#because Delta_SUM_Y = + - +++ -- ....so it is required to get the abs(Delta_y), This issue does not happen with x because the move is always 
#forward, and delta will be always positive. Or in this case negative because the robot is moving in negative axis...of lgsvl simulator....ie 10-0-10-15-16
final_list_delta_pose_y = []
for p_y in range(len(pose_y_array)-1):
    
    delta_pose_y_value = abs(pose_y_array.iloc[p_y+1]) - abs(pose_y_array.iloc[p_y])
    final_list_delta_pose_y.append(delta_pose_y_value)

delta_y_pose_abs_list = [abs(element) for element in final_list_delta_pose_y]


#delta_pose_x_pow = [x **2 for x in final_list_delta_pose_x]
#delta_pose_y_pow = [y **2 for y in final_list_delta_pose_y]

delta_pose_x_pow = [x **2 for x in delta_x_pose_abs_list]
delta_pose_y_pow = [y **2 for y in delta_y_pose_abs_list]


Sum_pose_components_List = list(map(add, delta_pose_x_pow, delta_pose_y_pow))

delta_pose_xy_module_list = np.sqrt(Sum_pose_components_List)


delta_y_pose_abs_list = [abs(element) for element in final_list_delta_pose_y]

#there is a small offset of x goal point, because of rviz, apollo manual interface to send to the goal point
#less than 1 unity of meter, for this reason I am going to round to next decimal, the DeltaX value
standard_x_goal_waypoint_planners = round(abs(pose_x_array.iloc[-1]-pose_x_array.iloc[0])/10)*10
Total_path_x_traveled_Public_Road = standard_x_goal_waypoint_planners #sum(delta_x_pose_abs_list)
Total_path_y_traveled_Public_Road = sum(delta_y_pose_abs_list)
Total_path_xy_traveled_Public_Road = math.sqrt(Total_path_x_traveled_Public_Road**2 + Total_path_y_traveled_Public_Road**2)

Cummulative_pose_xy_Public_Road = delta_pose_xy_module_list.sum()


Filter_by_cum_x_Df_Public_Road = pd.DataFrame({'Cummulative X': Cummulative_delta_x_pose_list,
                                   'Delta X':delta_x_pose_abs_list,
                                   'DElta Y':delta_y_pose_abs_list})



#-------------------------------------------------------------------------------------------------
#TASK 7 
#Join the Acceleration x, acceleration y and delta pose list, in order to separate the positive values (Power Hp from accel) and negative values (brake)
#Joined_accel_pose_df = pd.DataFrame({'Acceleration XY':linear_acceleration_xy_resultant_module_list,
 #                                    'Delta Pose XY': delta_pose_xy_module_list,
  #                                   'Delta Time': final_list_delta_seconds})

Join_acceleration_df = pd.DataFrame({'Acceleration X':Total_Accel_x,
                                     'Acceleration Y': Total_Accel_x,
                                     'Delta Pose XY': delta_pose_xy_module_list,
                                     'Delta Time': final_list_delta_seconds})


# REMOVING THE OUTLIERS: Before keeping the calculation of Force and Power it is requiered
#Identifyng the Outliers to remove
sns.boxplot(Join_acceleration_df["Acceleration X"])
sns.boxplot(Join_acceleration_df["Delta Pose XY"])


#Using IQR 
Q1 = Join_acceleration_df[["Acceleration X", "Acceleration Y", "Delta Pose XY"]].quantile(0.25)
Q3 = Join_acceleration_df[["Acceleration X", "Acceleration Y", "Delta Pose XY"]].quantile(0.75)
IQR = Q3 - Q1
#print(IQR)
Outliers_fst_quartile = Join_acceleration_df[["Acceleration X", "Acceleration Y", "Delta Pose XY"]] < (Q1 - 1.5 * IQR) 
Outliers_4th_quartile = Join_acceleration_df[["Acceleration X", "Acceleration Y", "Delta Pose XY"]] > (Q3 + 1.5 * IQR)

remove_Outliers_df_IQR_method = Join_acceleration_df[~((Join_acceleration_df < (Q1 - 1.5 * IQR)) |(Join_acceleration_df > (Q3 + 1.5 * IQR))).any(axis=1)]

Filtered_Parameters_Df = remove_Outliers_df_IQR_method


Acceleration_List = Filtered_Parameters_Df #[(Filtered_Parameters_Df[['Acceleration X']] > 0).all(1)]

Acceleration_xy_df = Acceleration_List[['Acceleration X', 'Acceleration Y']]


Acceleration_xy_Pow_df = Acceleration_xy_df.pow(2)
Sum_Acceleration_xy_pow_df = Acceleration_xy_Pow_df['Acceleration X'] + Acceleration_xy_Pow_df['Acceleration Y']
Resultant_Acceleration_xy_Sqrt_List = Sum_Acceleration_xy_pow_df**(1/2)
Resultant_accel_df = pd.DataFrame({'Acceleration XY':Resultant_Acceleration_xy_Sqrt_List})
Acceleration_rejoin_delta_df = pd.merge(Acceleration_List[['Delta Pose XY', 'Delta Time']], Resultant_accel_df, left_index=True, right_index=True)
Acceleration_xy_df_Public_Road = Acceleration_rejoin_delta_df

merged_accelx_resultant_xy_to_filter = pd.merge(Acceleration_xy_df,Acceleration_rejoin_delta_df,left_index=True, right_index=True)
Positive_Acceleration_resultant = merged_accelx_resultant_xy_to_filter[(merged_accelx_resultant_xy_to_filter[['Acceleration X']] > 0).all(1)]
Negative_Acceleration_resultant = merged_accelx_resultant_xy_to_filter[(merged_accelx_resultant_xy_to_filter[['Acceleration X']] < 0).all(1)]



#Accel Pos Resultant
Resultant_Accel_xy_Sqrt_List = Positive_Acceleration_resultant['Acceleration XY']
#Brake List  to get friction Energy = Acceleration X = Negative
Resultant_Brake_xy_Sqrt_List = Negative_Acceleration_resultant['Acceleration XY']


#-----------------------------------------------------------------------------------------------------------------
#TASK 8: CALCULATION OF TOTAL FORCE DELIVERED BY THE VEHICLE MOTOR
#Separate opposite force (brake) from positive (accel)
# F = M * a
Force_Acceleration_df = Resultant_Accel_xy_Sqrt_List.apply(lambda x: x*M)
Force_Brake_df = Resultant_Brake_xy_Sqrt_List.apply(lambda x: x*M)

#You cannot drop the forces into negative and positive, instead you must drop the action = W, otherwise we are having bigger deltas = x3-x1, because you dropped x2 with the negative force, 
#and Force 3 will multiply x3-x1, but should multiply x2-x2 and this will increase the power at end



#TASk 9: Calcualte instantaneous Action= work = N.m = Joule

Action_Acceleration_df = Force_Acceleration_df * Positive_Acceleration_resultant['Delta Pose XY'] 
Action_Brake_df = Force_Brake_df * Negative_Acceleration_resultant['Delta Pose XY'] 



#Taks 10: Calculate Power in Hp

Power_Acceleration_df_watts = Action_Acceleration_df / Positive_Acceleration_resultant['Delta Time']
Power_Brake_df_watts = Action_Brake_df / Negative_Acceleration_resultant['Delta Time']

Power_Acceleration_df_hp = Power_Acceleration_df_watts.apply(lambda x: x/745.699872)
Power_Brake_df_hp = Power_Brake_df_watts.apply(lambda x: x/745.699872)


#Mean = Average
Mean_Public_Road_Acceleration_power = Power_Acceleration_df_hp.mean()
Median_Public_Road_Acceleration_power = Power_Acceleration_df_hp.median()
#deviation
std_dev_Public_Road_acceleration = stdev(Power_Acceleration_df_hp)
variance_Public_Road_acceleration = variance(Power_Acceleration_df_hp)
#CV variation coefficient = Standard deviation / mean. If CV < 1 indicates a low variation, however for non-normal distribution the standard deviation does not mean anything, also the CV
#Source: https://www.researchgate.net/post/What-do-you-consider-a-good-standard-deviation
CV_Accel_Power = std_dev_Public_Road_acceleration / Mean_Public_Road_Acceleration_power

#Mean/Average?/Median Brake Power (BHP)
Mean_Public_Road_brake_power = Power_Brake_df_hp.mean()
Median_Public_Road_brake_power = Power_Brake_df_hp.median()

std_dev_Public_Road_brake = stdev(Power_Brake_df_hp)
variance_Public_Road_acceleration = variance(Power_Brake_df_hp)
#CV variation coefficient = Standard deviation / mean. If CV < 1 indicates a low variation, however for non-normal distribution the standard deviation does not mean anything, also the CV
#It just says how much your data is spread from the mean...but does not say if it is wrong!
CV_Brake_Power = std_dev_Public_Road_brake / Mean_Public_Road_brake_power




#-------------------------------------MECHANIC ENERGY CALCULATION-----------------------------


time_stamp_array_vel = df_synchronized_imu['TimeStamp']
linear_accel_x_array_vel = df_synchronized_imu['LinearAcceleration_x_y']
linear_accel_y_array_vel = df_synchronized_imu['LinearAcceleration_x_y']

final_list_vel = []
for i in range(len(time_stamp_array_vel)):
    
    time_value = time_stamp_array_vel.iloc[i]
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
    final_list_vel.append(human_readable)


final_list_vel_delta_seconds = []
for t in range(len(final_list_vel)-1):
    
    delta_t = final_list_vel[t+1] - final_list_vel[t]
    #print(delta_t)
    
    #In miliseconds to seconds conversion
    delta_ms_new = ( delta_t / dt.timedelta(milliseconds=1) ) *1e-3
    #print(delta_ms_new)
    final_list_vel_delta_seconds.append(delta_ms_new)


#DELTA LINEAR ACCELERATION & VELOCITY CALCULATION FROM DELTA ACCELERATION
#For X axis
# ΔV_linear_x =  Δa_linear_accel_x * Δt

final_list_delta_linear_vel_accel_x = []
for a_lin_x in range(len(linear_accel_x_array_vel)-1):
    delta_a_lin_x = linear_accel_x_array_vel.iloc[a_lin_x+1] - linear_accel_x_array_vel.iloc[a_lin_x]
    final_list_delta_linear_vel_accel_x.append(delta_a_lin_x)
    

delta_new_velocity_x_list = [a * b for a,b in zip(final_list_delta_linear_vel_accel_x, final_list_vel_delta_seconds)]


final_list_delta_linear_vel_accel_y = []
for a_lin_y in range(len(linear_accel_y_array_vel)-1):
    delta_a_lin_y = linear_accel_y_array_vel.iloc[a_lin_y+1] - linear_accel_y_array_vel.iloc[a_lin_y]
    final_list_delta_linear_vel_accel_y.append(delta_a_lin_y)
    #test average


delta_new_velocity_y_list = [c * d for c,d in zip(final_list_delta_linear_vel_accel_y, final_list_vel_delta_seconds)]


Vel_x_List = delta_new_velocity_x_list #delta_velocity_x_list  #velocity_forward_median 
Vel_y_List = delta_new_velocity_y_list #delta_velocity_y_list



#Get new angular vel data
angular_vel_z_array_vel = df_synchronized_imu['AngularVelocity_z_y']

#test
vel_final_list_delta_angular_velocity_z = []
for v_z in range(len(angular_vel_z_array_vel)-1):
    angular_velocity = angular_vel_z_array_vel.iloc[i]
    delta_v_z = angular_vel_z_array_vel.iloc[v_z+1] - angular_vel_z_array_vel.iloc[v_z]
    absolute_diff = delta_v_z  # To calculate the power does not matter the direction of acceleration, just the module how much the car is taking of energy 
    vel_final_list_delta_angular_velocity_z.append(absolute_diff)
    
#------------------------------------------------------------------------------------------------------------------    

#TASK3: Get the Rate of Angular Acceleration = θ = Δω / Δt 
vel_acceleration_z_list = [i /j for i,j in zip(vel_final_list_delta_angular_velocity_z, final_list_vel_delta_seconds)]


#------------------------------------------------------------------------------------------------------------------

#TASK4: Calculate the Total Acceleration = Anet = sqrt (atencial² + acentripetal²)
# A) Tangential Acceleration

vel_orientation_yaw_z_array = df_synchronized_imu['EulerAngles_z']

vel_Steering_yaw_list = []
for steer in range(len(vel_orientation_yaw_z_array)-1):
#At first the vehicle is turning CCW to avoid obstacle, therefore the Delta value is + (convetion)
#Then it returns to the Lane, turning CW and the new yaw/steer values are negative. So a if condition must be done        
#In contrast with AUTOWARE data, here in APollo the yaw angle starts at 90 deg as reference so the IMU never get negative values
#For this reason the code will be simple with if condition    

    steer_yaw_value = vel_orientation_yaw_z_array.iloc[steer]
 #   if steer_yaw_value < 0: # CCW = + yaw
    delta_steer_value = steer_yaw_value - vel_orientation_yaw_z_array.iloc[0] #because the first value is starting with negative value (-3 = -178 deg) and next is (-2 turning CCW) = the difference is 1 rad, so -2 - (-3) = -2 +3 = 1 rad
    absolute_delta_steer = delta_steer_value #The same here as angular velocity, does not matter if the car rotated CCW or CW to calculate the power...
 #   else:
        #delta_steer_value = steer_yaw_value + orientation_yaw_array[0]
        #absolute_delta_steer = abs(delta_steer_value)#The module will make easy later to calculate the ICR (positived always) and Tangential Acceleration
    vel_Steering_yaw_list.append(absolute_delta_steer)


#calculate below the tangent(Steer = alpha =yaw) array 
vel_tan_yaw_list = []


for tan_a in range(len(vel_Steering_yaw_list)):
    tan_yaw_value = math.tan(vel_Steering_yaw_list[tan_a])
    vel_tan_yaw_list.append(tan_yaw_value)



#Calculate the Instantaneous Center of Rotation (ICR)
# R = ICR = tan(alpha) * L , where L is the vehicle's wheelbase
vel_ICR_List = []
for rad in range(len(vel_tan_yaw_list)):
    icr_value = L * vel_tan_yaw_list[rad]
    vel_ICR_List.append(icr_value)


#Finally the tangential acceleration is given by 
#at = θ x R
Vel_Tangential_Acceleration_List =  [k * l for k,l in zip(vel_acceleration_z_list, vel_ICR_List)]


#test



#Convert Rotational Velocity To linear

#vrot = L*(ω/tan α)

Vrot_List_aux =  [l /m for l,m in zip(vel_final_list_delta_angular_velocity_z, vel_tan_yaw_list)]
Vrot_Final_List = [element * L for element in Vrot_List_aux] 

#Need to convert the Vrot in ax and ay components, using the steer angle, or yaw angle

#auxiliary matrixof cos and sin
Steering_yaw_list_cos_list = np.cos(vel_Steering_yaw_list)
Steering_yaw_list_sin_list = np.sin(vel_Steering_yaw_list)


Vrot_linear_x_component =  [m * n for m,n in zip(Vrot_Final_List, Steering_yaw_list_cos_list)]
Vrot_linear_y_component =  [o * p for o,p in zip(Vrot_Final_List, Steering_yaw_list_sin_list)]

#SUM the rotational to linear

V_sum_x_components = [a + b for a,b in zip(Vel_x_List, Vrot_linear_x_component)]
V_sum_y_components = [a + b for a,b in zip(Vel_y_List, Vrot_linear_y_component)]

#Join Vel to DF to filter Acceleration and Brake Power Consumption
Joined_velocity_xy_df = pd.DataFrame({"Velocity X":V_sum_x_components,
                         "Velocity Y": V_sum_y_components,
                         'Delta Time': final_list_vel_delta_seconds})

#Filtering the Outliers
#Filter the Outliers Velocity

#Using IQR 
Q1_v = Joined_velocity_xy_df[["Velocity X", "Velocity Y", "Delta Time"]].quantile(0.25)
Q3_v = Joined_velocity_xy_df[["Velocity X", "Velocity Y", "Delta Time"]].quantile(0.75)
IQR_v = Q3 - Q1
#print(IQR)
Outliers_fst_quartile = Joined_velocity_xy_df[["Velocity X", "Velocity Y", "Delta Time"]] < (Q1 - 1.5 * IQR_v) 
Outliers_4th_quartile = Joined_velocity_xy_df[["Velocity X", "Velocity Y", "Delta Time"]] > (Q3 + 1.5 * IQR)

remove_Outliers_velocity_df_IQR_method = Joined_velocity_xy_df[~((Joined_velocity_xy_df < (Q1_v - 1.5 * IQR_v)) |(Joined_velocity_xy_df > (Q3_v + 1.5 * IQR_v))).any(axis=1)]

Filtered_Parameters_velocity_Df = remove_Outliers_velocity_df_IQR_method

#Separate brake from accel
Positive_Velocity_df = Filtered_Parameters_velocity_Df[(Filtered_Parameters_velocity_Df[['Velocity X']] > 0).all(1)]
Negative_Velocity_df = Filtered_Parameters_velocity_Df[(Filtered_Parameters_velocity_Df[['Velocity X']] < 0).all(1)]
#get The Resultant
Total_positive_velocity_xy_df = Positive_Velocity_df[['Velocity X', 'Velocity Y']]
Positive_velocity_xy_Pow_df = Total_positive_velocity_xy_df.pow(2)
Sum_Positive_velocity_xy_pow_df = Positive_velocity_xy_Pow_df['Velocity X'] + Positive_velocity_xy_Pow_df['Velocity Y']
Resultant_Positive_Velocity_xy_Sqrt_df = Sum_Positive_velocity_xy_pow_df**(1/2)



list_aux_delta_time = Positive_Velocity_df['Delta Time'].to_list()
list_aux_res_vel = Resultant_Positive_Velocity_xy_Sqrt_df.to_list()

get_back_complete_df = pd.DataFrame({"Resultant Velocity":list_aux_res_vel,
                                     "Delta Time": list_aux_delta_time})

#Filtering the Ouliers again

#Filter Again the Outliers
Q1_res_v = get_back_complete_df[["Resultant Velocity", "Delta Time"]].quantile(0.25)
Q3_res_v = get_back_complete_df[["Resultant Velocity", "Delta Time"]].quantile(0.75)
IQR_vres = Q3_res_v - Q1_res_v
#print(IQR)
Outliers_fst_quartile_Q1_res = get_back_complete_df[["Resultant Velocity", "Delta Time"]] < (Q1 - 1.5 * IQR_vres) 
Outliers_4th_quartile_Q1_res = get_back_complete_df[["Resultant Velocity", "Delta Time"]] > (Q3 + 1.5 * IQR_vres)

remove_Outliers_velocity_res_df_IQR_method = get_back_complete_df[~((get_back_complete_df < (Q1_res_v - 1.5 * IQR_vres)) |(get_back_complete_df > (Q3_res_v + 1.5 * IQR_vres))).any(axis=1)]

Filtered_Parameters_velocity_res_v_Df = remove_Outliers_velocity_res_df_IQR_method

#calculate Mechanica Energy Variation  ΔE = mVf² - mV0² 

delta_mechanic_energy = (M * Filtered_Parameters_velocity_res_v_Df["Resultant Velocity"])/2
delta_mechanic_list_aux = delta_mechanic_energy.to_list()
delta_mechanic_column_rename = pd.DataFrame({'Mechanic Energy (ΔE)': delta_mechanic_list_aux})
delta_time_aux = Filtered_Parameters_velocity_res_v_Df["Delta Time"].to_list()
delta_time_rename = pd.DataFrame({'Delta Time': delta_time_aux})

Power_Consumption_Watts_df = delta_mechanic_column_rename["Mechanic Energy (ΔE)"] / delta_time_rename["Delta Time"]
Power_Consumption_Horsepower_df = Power_Consumption_Watts_df / 745.699872

Delta_cumulative_time = delta_time_rename["Delta Time"].cumsum()
aux_delta_time_list = Delta_cumulative_time.to_list()
New_Delta_cumulative_time = pd.DataFrame({'Cummulative Time Variation (ΔT)':aux_delta_time_list})
plot_df = pd.concat([delta_mechanic_column_rename, New_Delta_cumulative_time], axis=1)
drop_nan_plot_df = plot_df.dropna()

#Plot To find the function which is suitable to curve

x = drop_nan_plot_df["Cummulative Time Variation (ΔT)"]                   
y = drop_nan_plot_df["Mechanic Energy (ΔE)"] 

ax1 = drop_nan_plot_df.plot.scatter(x="Cummulative Time Variation (ΔT)", y="Mechanic Energy (ΔE)") #Actually is mechanic energy

plt.scatter(x,y)




f = CubicSpline(x,y, bc_type='not-a-knot')
x_new = np.linspace(x.min(), x.max())
y_new = f(x_new)

#Der = CubicSpline.__call__(x_new, y_new, nu=1)
#Cubic_Spline_derivative = CubicSpline.derivative(f, nu=1)

#plt.plot(x_new,y_new)
#plt.plot(x,y,'ro', lw=3)



'''
spl = UnivariateSpline(x,y)

#Smoothing parameter
xs = np.linspace(min(x), max(x))
#Change the amount of smoothing factor
spl.set_smoothing_factor(0.5)

plt.scatter(x,y)
plt.plot(xs, spl(xs), 'g', lw=3)
'''


#THE DERIVATIVE RETURNS ERROR

'''
#plot derivative
Spline_Derivatives = spl.derivatives(1) #order of derivative
Derivative_Values_Sum = Spline_Derivatives.sum()

Instantaneous_Hp_Df = Derivative_Values_Sum / 745.699872

Mean_Power_Consumption = Instantaneous_Hp_Df.mean()
Median_Power_Consumption = Instantaneous_Hp_Df.median()
'''


'''
m,b = np.polyfit(x,y,1)
plt.plot(x, m*x + b)


slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
res= stats.linregress(x,y)
plt.plot(x, y, 'o', label='original data')

X = sp.symbols('x')
#Find the equation = Y = mx + b
Function_x = slope*X + intercept 

#Derivate the function to get the Power in Watts
Simple_derivative = sp.Derivative(Function_x, X)
 


#Get the coefficients found above to apply to Dataframe, and find each instataneous Power Value
derivative_coeff_aux = Simple_derivative._args[0]
String_Simple_Derivative = str(derivative_coeff_aux)
coeff_array = re.findall(r'[0-9]+', String_Simple_Derivative)
first_coef_round = float(coeff_array[0])
last_coef_of_x_round = float(coeff_array[2])


Instantaneous_Power_Watts_Df =  first_coef_round + (-last_coef_of_x_round *delta_time_rename["Delta Time"])
#Instantaneous_Power_Watts_Df =  16.1633672046932 + (- 0.603436831643907 *Delta_Time_df["Delta Time"])
Instantaneous_Hp_Df = Instantaneous_Power_Watts_Df / 745.699872


#It does not make sense to sum all the Power Spent by the motor in each waypoint, it is such as sum the velocity of an athlete at each waypoint of a trajectory and say that in the end
#the athlete runned with 300 km/h for example, but the average at each waypoint makes sense where the athlete runned 10 km/h for example in a 100 meters run
#Total_amount_of_Integral = Instantaneous_Hp_Df.sum()

Mean_Power_Consumption = Instantaneous_Hp_Df.mean()
Median_Power_Consumption = Instantaneous_Hp_Df.median()

#Finally the standard deviation must be calculated and added to the mean
standard_deviation_Power_Consumption = stdev(Instantaneous_Hp_Df)
variance_Power_Consumption = variance(Instantaneous_Hp_Df)

Final_Power_Op_mean_plus_stdev = Mean_Power_Consumption + standard_deviation_Power_Consumption

'''
