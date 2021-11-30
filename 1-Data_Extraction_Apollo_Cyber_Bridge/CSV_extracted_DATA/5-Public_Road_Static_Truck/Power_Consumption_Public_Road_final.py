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

#The only difference from Apollo to Autoware Script is that Apollo get also linear_y velocity data




#Reading from IMU beacasue it has all the needed data to calculate power
#It has not linear velocity but has acceleration, so it is just extract the integral of acceeleration
df_raw_imu = pd.read_csv(r'Public_Road_raw_imu_message_Static_Truck.csv')
df_imu = pd.read_csv(r'Public_Road_corrected_imu_messages_Static_Truck.csv')
# Teh csv data above are not synchronized...however both of data are required
#because the the raw_imu contain linear valocity and accel data, and the imu contain the Orientation yaw angles in Euler...
#Therefore it is needed to merge both on TimeStamp data to drop unmatched TimeStamps
df_velocity = pd.read_csv(r'Public_Road_chassis_Static_Truck.csv')


df_synchronized_imu = df_imu.merge(df_raw_imu, on="TimeStamp", copy=False)
df_synchronized_imu.to_csv('Merged_imu.csv')

#df_pose = pd.read_csv(r'current_pose_op.csv')
#df_yaw_angle = pd.read_csv(r'converted_quaternions_to_Euler.csv')

#From manual, still not from LGSVL Unity 3D, just to test
global L
L = float(2.917986)   #float(2.72796) #wheelbase in meters of Lincoln MKZ model from LGSVL UNITY3D
M = int(2120) # Kg taken in LGSVL support - unix measurement, Same for both vehicles

#Check if the csv Counters-Timestamp from files above are aligned, if not, drop the lines manually to synchronize
# and then replace "," to "." because after drop operation it loses the format...and do not work the code..

#Get TimeStamp
time_stamp_array = df_synchronized_imu['TimeStamp']
#Get LinearAcceleration_X -- After merge the columns name changed from 'LinearAcceleration_x' to 'LinearAcceleration_x_y'
#Must check in the new csv the columns to match exactly
linear_accel_x_array = df_synchronized_imu['LinearAcceleration_x_y']
#Get LinearAcceleration_Y
linear_accel_y_array = df_synchronized_imu['LinearAcceleration_y_y']
#Get Angular_velocity_X 
angular_vel_x_array = df_synchronized_imu['AngularVelocity_x_y']
#Get Angular_velocity_Y 
angular_vel_y_array = df_synchronized_imu['AngularVelocity_y_y']
#Get Angular_Velocity_Z
angular_vel_z_array = df_synchronized_imu['AngularVelocity_z_y']
#Get Yaw Angle 
orientation_yaw_z_array = df_synchronized_imu['EulerAngles_z']
#Get velocity data: Resultant Forward x,y
velocity_forward_xy_array = df_velocity['Speed_mps']


#---------------------------------------------------------------------------

#TASKS FOR POWER CALCULATION PSEUDO TUTORIAL

#Delta time Extraction and Conversion to seconds into a Time_List

#TASK 1: CONVERT TIME MACHINE TO HUMAN AND GET VELOCITY VARIATION AND ACCELERATION IN X AXIS



final_list = []
for i in range(len(time_stamp_array)):
    
    time_value = time_stamp_array[i]
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


#DELTA LINEAR ACCELERATION & VELOCITY CALCULATION FROM DELTA ACCELERATION
#For X axis
# ΔV_linear_x =  Δa_linear_accel_x * Δt

final_list_delta_linear_accel_x = []
for a_lin_x in range(len(linear_accel_x_array)-1):
    delta_a_lin_x = linear_accel_x_array[a_lin_x+1] - linear_accel_x_array[a_lin_x]
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
    delta_a_lin_y = linear_accel_y_array[a_lin_y+1] - linear_accel_y_array[a_lin_y]
    final_list_delta_linear_accel_y.append(delta_a_lin_y)
    #test average
final_value_average_accel_y = (np.nansum(final_list_delta_linear_accel_y)) / len(final_list_delta_linear_accel_y)


delta_velocity_y_list = [c * d for c,d in zip(final_list_delta_linear_accel_y, final_list_delta_seconds)]
delta_velocity_y_value_average = np.nansum(delta_velocity_y_list) /len(delta_velocity_y_list)

#GET THE MODULE OF THE ACCELERATION in X and Y that the motor  is realizing
# At = sqrt(a_lin_x² + a_lin_y²)
#Calculation a²
linear_accel_pow_x_list = [e **2 for e in final_list_delta_linear_accel_x]
linear_accel_pow_y_list = [f **2 for f in final_list_delta_linear_accel_y]


linear_acceleration_xy_pow_sum_list = [g + h for g,h in zip(linear_accel_pow_x_list, linear_accel_pow_y_list)]
linear_acceleration_xy_resultant_module_list = np.sqrt(linear_acceleration_xy_pow_sum_list)
linear_accel_xy_average = np.nansum (linear_acceleration_xy_resultant_module_list) / len(linear_acceleration_xy_resultant_module_list)
#test calculation of average acceleration module



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
    angular_velocity = angular_vel_z_array[i]
    delta_v_z = angular_vel_z_array[v_z+1] - angular_vel_z_array[v_z]
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

    steer_yaw_value = orientation_yaw_z_array[steer]
 #   if steer_yaw_value < 0: # CCW = + yaw
    delta_steer_value = steer_yaw_value - orientation_yaw_z_array[0] #because the first value is starting with negative value (-3 = -178 deg) and next is (-2 turning CCW) = the difference is 1 rad, so -2 - (-3) = -2 +3 = 1 rad
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


Accel_x_component_Power_List = [v ** 2 for v in acceleration_x_from_rotational_net_List]
Accel_y_component_Power_List = [w ** 2 for w in acceleration_y_from_rotational_net_List]

Sum_accel_components_List = list(map(add, Accel_x_component_Power_List, Accel_y_component_Power_List)) 
Accel_xy_resultant_components = np.sqrt(Sum_accel_components_List)


#Accel_xy_resultant_components_average = (np.nansum(Accel_xy_resultant_components)) / len(Accel_xy_resultant_components)
#Accel_xy_resultant_components_median = np.median(Accel_xy_resultant_components)


Accel_xy_resultant_components_average = np.sqrt(acceleration_x_from_rot_anet_average**2 + acceleration_y_from_rot_anet_average**2)


#-----------------------------------------------------------------------------------------------------------------

#TASK 6: CALCULATION OF TOTAL FORCE DELIVERED BY THE VEHICLE MOTOR
# Ft = Flinear + Frotational ---> Flinear = Mass * ( a_linear_x + a_linear_x) ||| Frotational = Mass * anet ----> Ft = Mass * (a_linear_X + anet)
#The only difference here from Autoware is that Apollo displayed the linear_y component. 
#Therefore the acceleration_linear_Total = linear_acceleration_xy_resultant_module_list must be used

#Total_Acceleration_Linear_plus_Rotational_List = [sum(k) for k in zip(linear_acceleration_xy_resultant_module_list, Anet_List)]

Total_Acceleration_Linear_plus_Rotational_List = [ k + l for k,l in zip(linear_acceleration_xy_resultant_module_list, Accel_xy_resultant_components)]
Total_Acceleration_Linear_plus_Rotational_List = linear_accel_xy_average + Accel_xy_resultant_components_average


#Total_Accel_Lin_Rot_Average = np.nansum(Total_Acceleration_Linear_plus_Rotational_List) / len(Total_Acceleration_Linear_plus_Rotational_List)
#Total_Accel_Lin_Rot_Median = np.median(Total_Acceleration_Linear_plus_Rotational_List)


Ft_Value_Average = M *Total_Acceleration_Linear_plus_Rotational_List  #(linear_accel_xy_average + Total_Accel_Lin_Rot_Median)

#-----------------------------------------------------------------------------------------------------------------
#TASK 7: FIND THE TOTAL VELOCITY OF THE VEHICLE = VT = V_linear_X + Vrotational_converted_in_x_y_components

Vel_Linear_xy_total = velocity_forward_average  #velocity_forward_median 


#vrot = L*(ω/tan α)

Vrot_List_aux =  [l /m for l,m in zip(final_list_delta_angular_velocity_z, tan_yaw_list)]
Vrot_Final_List = [element * L for element in Vrot_List_aux] 

#Need to convert the Vrot in ax and ay components, using the steer angle, or yaw angle

#auxiliary matrixof cos and sin
Steering_yaw_list_cos_list = np.cos(Steering_yaw_list)
Steering_yaw_list_sin_list = np.sin(Steering_yaw_list)


Vrot_linear_x_component =  [m * n for m,n in zip(Vrot_Final_List, Steering_yaw_list_cos_list)]
Vrot_linear_y_component =  [o * p for o,p in zip(Vrot_Final_List, Steering_yaw_list_sin_list)]

#FINAL STEP: Calculate the Total Velocity
# Vt =   sqrt [ (vel_linear_x + Vrot_Linear_x_component)² + (vel_rot_linear_y_component)² ]
#REMOVING INF and NULL values from arrays

Vrot_linear_x_component_mask = np.ma.masked_array(Vrot_linear_x_component, ~np.isfinite(Vrot_linear_x_component)).filled(0)
Vrot_linear_y_component_mask = np.ma.masked_array(Vrot_linear_y_component, ~np.isfinite(Vrot_linear_y_component)).filled(0)

#Removing opposite velocity values that does not put up the Power Consumption of engine, instead construct the BHP (brake horspower)
Vrot_linear_x_drop_negative = np.array( [num for num in Vrot_linear_x_component_mask if num>0])
Vrot_linear_y_drop_negative = np.array( [num for num in Vrot_linear_y_component_mask if num>0])


#Average of vrot_linear
#Vrot_linear_x_component_average =  (np.nansum(Vrot_linear_x_drop_negative)) / len(Vrot_linear_x_drop_negative)
#Vrot_linear_y_component_average = (np.nansum(Vrot_linear_y_drop_negative)) / len(Vrot_linear_y_drop_negative)
Vrot_linear_x_component_median = np.median(Vrot_linear_x_drop_negative)
Vrot_linear_y_component_median = np.median(Vrot_linear_y_drop_negative)

#The average is returning to high values, probably due high variation on sensor measurements (Noise), therefore wrong. Take the median in this
#case is a suitable solution, as it is has almost the same effect of taking the average

#Total Components Values
Power_Linear_Vel_x_List = Vrot_linear_x_component_median**2 #Vrot_linear_x_component_average **2
Power_Linear_Vel_y_List = Vrot_linear_y_component_median**2 #Vrot_linear_y_component_average **2
#Need to sum the vel above 
Total_vel_rot_converted_xy_linear = math.sqrt(Power_Linear_Vel_x_List + Power_Linear_Vel_y_List)


#Sum the Power of 2 components (x,y) 
Velocity_Total_Sum = Vel_Linear_xy_total + Total_vel_rot_converted_xy_linear

#------------------------------------------------------------------------------------
#TASK 8 CALCULATE THE POWER CONSUMPTION: P = F x V = Ft_List * Velocity_Total_List
Power_Consumption_watts = Ft_Value_Average * Velocity_Total_Sum

Total_Power_Consumption_Hp_Public_Road_planner = Power_Consumption_watts / 745.699872
#Conversion to HORSWPOWER (HP) : 1 hp(I) = 745.699872 W ---> P(hp) = P(W) / 745.699872


#FINALLY THE OPPOSITE FORCES (DRAG AND FRICTION) MUST BE ADDED - SUBTRACTED FROM FIRST CALCULATED FORCE


















