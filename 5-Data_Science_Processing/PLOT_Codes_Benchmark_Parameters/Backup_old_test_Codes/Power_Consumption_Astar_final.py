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

df_vel = pd.read_csv(r'current_velocity_astar.csv')
df_pose = pd.read_csv(r'current_pose_astar.csv')
df_yaw_angle = pd.read_csv(r'astar_converted_quaternions_to_Euler.csv')

#From manual, still not from LGSVL Unity 3D, just to test
global L
L = float(2.834891)   #float(2.72796) #wheelbase in meters of Jaguar Xe model
M = float(2120) # Kg taken in LGSVL support - unix measurement

#Check if the csv Counters-Timestamp from files above are aligned, if not, drop the lines manually to synchronize
# and then replace "," to "." because after drop operation it loses the format...and do not work the code..


#Get Time_Stamp columns in UInix Epoch (Machine Format)
time_stamp_array = df_pose["%time"]
#Get the column of velocity data in x(m/s)
vel_linear_x_array = df_vel["field.twist.linear.x"]
#Get the columns of angular velocity in z(rad/s)
vel_angular_z_array = df_vel["field.twist.angular.z"]
#There is NOT Linear velocity in Y, because the car has nonholomic constraint, that's why need to convert Z in x and y velocity later
orientation_yaw_array = df_yaw_angle["yaw"]

#------------------------------------------------------------------------------------------------------------------

#TASKS OF POWER CALCULATION TUTORIAL

#TASK 1: CONVERT TIME MACHINE TO HUMAN AND GET VELOCITY VARIATION AND ACCELERATION IN X AXIS

# Loop inside the Unix Epoch TimeStamp List To convert to Humna Readable Format
#Time List Extraction to Human Format

final_list = []
for i in range(len(time_stamp_array)):
    
    time_value = time_stamp_array[i]
    human_readable = datetime.fromtimestamp(time_value // 1e9)
    string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
    
    str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
    #required to drop the last 6 characters related to microseconds otherwise next line will not process it
    str_human_time = str_human_time[:-6]

    human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
    #print(human_time_float)
    final_list.append(human_time_float)

#print(human_time_float)

#Delta time Extraction and Conversion to seconds into a Time_List
final_list_delta_seconds = []
for t in range(len(final_list)-1):
    
    #date_time_value = final_list[i]
    #print(date_time_value)
    delta_t = final_list[t+1] - final_list[t]
    
    #In miliseconds to seconds conversion
    delta_ms = ( delta_t / dt.timedelta(milliseconds=1) ) *1e-3
    final_list_delta_seconds.append(delta_ms)
    
    
    
#Calculate the Delta velocity of Time n - Time(n-1)
final_list_delta_velocity_x = []
for v_x in range(len(vel_linear_x_array)-1):
    #v_x_value = vel_linear_x_array[v_x]
    #print(v_x_value)
    delta_v_x = vel_linear_x_array[v_x+1] - vel_linear_x_array[v_x]
    final_list_delta_velocity_x.append(delta_v_x)
        
        
#Extract the Acceleration  = Velocity_linear_x_Array / Human Radable Time in Seconds 
acceleration_x_list = [a /b for a,b in zip(final_list_delta_velocity_x, final_list_delta_seconds)]
#Negative values mean that the car is braking, therefore does not mean the car is generating power in these 
#times and will prejudice the average value calculation pulling to small value
drop_brake_opposite_accel_values_List = np.array( [num for num in acceleration_x_list if num>=0])
drop_brake_opposite_velocity_list = np.array( [num for num in final_list_delta_velocity_x if num>=0])


acceleration_x_value_average = (np.nansum(drop_brake_opposite_accel_values_List)) / len(drop_brake_opposite_accel_values_List)


average_delta_velocity_x = (np.nansum(drop_brake_opposite_velocity_list)) / len(final_list_delta_velocity_x)
#median_delta_velocity_x = np.median(drop_brake_opposite_velocity_list)

#THE DELTA VELOCITY WILL NOT BE USEAD AS VLOCITY AVERAGE, HOWEVER JUST THE POSITIVE VALEUS OF VELOCUTY ARRAY (NOT DELTA VELOCITY)
#Because we wish to figure out how much the engine has worked to accelerate the car, not to stop it

positive_velocity_x_array = np.array( [num for num in vel_linear_x_array if num>0])
average_velocity_x = np.nansum(positive_velocity_x_array) / len(positive_velocity_x_array)
#median_velocity_x = np.median(positive_velocity_x_array)


#Acceleration_x_Absolute_List = np.abs(acceleration_x_list)


#------------------------------------------------------------------------------------------------------------------

#TASK2: GET THE ANGULAR VELOCITY VARIATION = Δω = Rad/s
#CCW is positive and CW is negative - However it is not required if condition, as the difference will always show if the move is mainly CCW or CW

final_list_delta_angular_velocity_z = []
for v_z in range(len(vel_angular_z_array)-1):
    angular_velocity = vel_angular_z_array[v_z]
    delta_v_z = vel_angular_z_array[v_z+1] - vel_angular_z_array[v_z]
    absolute_diff = delta_v_z  # To calculate the power does not matter the direction of acceleration, just the module how much the car is taking of energy 
    final_list_delta_angular_velocity_z.append(absolute_diff)
     
#------------------------------------------------------------------------------------------------------------------    

#TASK3: Get the Rate of Angular Acceleration = θ = Δω / Δt 
acceleration_z_list = [c /d for c,d in zip(final_list_delta_angular_velocity_z, final_list_delta_seconds)]

#------------------------------------------------------------------------------------------------------------------

#TASK4: Calculate the Total Acceleration = Anet = sqrt (atencial² + acentripetal²)
# A) Tangential Acceleration
    
#Get the alpha ( that is the Orientation in Z in each time, the Yaw angle)
#ROS returns this list in Quaternion, then It is neeeded the conversion in Yaw angle, which was done in the code
#quaternion_To_euler.py afer extract ROSBAG topics and convert them. Just needed drop some line and first columns  for synschornization purposes with velocity topic

#df_yaw_angle = array of seteering angle =yaw angle for Ackermann steer, this array is the steer angle of each instant
#An array of 696 elements. However to match the other arrays the first element must be fixed and subtracted always
# from the next steering values (It is how much the angle is steering referenced with the first vehicle orientation)
Steering_yaw_list = []
for steer in range(len(orientation_yaw_array)-1):
#At first the vehicle is turning CCW to avoid obstacle, therefore the Delta value is + (convetion)
#Then it returns to the Lane, turning CW and the new yaw/steer values are negative. So a if condition must be done        
    steer_yaw_value = orientation_yaw_array[steer]
    delta_steer_value = steer_yaw_value - orientation_yaw_array[0] #because the first value is starting with negative value (-3 = -178 deg) and next is (-2 turning CCW) = the difference is 1 rad, so -2 - (-3) = -2 +3 = 1 rad
    Steering_yaw_list.append(delta_steer_value) #The same here as angular velocity, does not matter if the car rotated CCW or CW to calculate the power...



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
Tangential_Acceleration_List =  [e * f for e,f in zip(acceleration_z_list, ICR_List)]

#Task4 
# B) Calculating the Centripetal Acceleration --> ac = r*ω² , where r = ICR and w = angular velocity
#w²    
Power_delta_angular_vel_z = [f ** 2 for f in final_list_delta_angular_velocity_z]
#ac
accel_centripetal_list = [g * h for g,h in zip(ICR_List, Power_delta_angular_vel_z)]

#TASK 5 Calculation of Total Acceleration
#TOTAL ACCELERATION = Anet = sqrt (acentripetal² + at²)

Acentripetal_Power_List = [i ** 2 for i in accel_centripetal_list]
Atangential_Power_List = [j ** 2 for j in Tangential_Acceleration_List]

Sum_accel_List = list(map(add, Acentripetal_Power_List, Atangential_Power_List)) 
#Sum_accel_List = [sum(k) for k in zip(Acentripetal_Power_List, Atangential_Power_List)]

Anet_List = np.sqrt(Sum_accel_List)

#Line below is not necessary anymore, because the anet must be decomposed in x and y components
#Total_accel_average_value = (np.nansum(Anet_List )) / len(Anet_List )


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

drop_negative_opposite_accel_x = np.array( [num for num in acceleration_x_from_rotational_net_List if num>=0])
acceleration_x_from_rot_anet_average = (np.nansum(drop_negative_opposite_accel_x)) / len(drop_negative_opposite_accel_x)

acceleration_y_from_rotational_net_List = []

for accel_y_rot in range(len(Theta_decomposition_angle_List)):
    accel_y_component_value = Anet_List[accel_y_rot] * (math.cos(Theta_decomposition_angle_List[accel_y_rot]))
    acceleration_y_from_rotational_net_List.append(accel_y_component_value)

drop_negative_opposite_accel_y = np.array( [num for num in acceleration_y_from_rotational_net_List if num>=0])
acceleration_y_from_rot_anet_average = (np.nansum(drop_negative_opposite_accel_y)) / len(drop_negative_opposite_accel_y)



Accel_x_component_Power_List = [v ** 2 for v in acceleration_x_from_rotational_net_List]
Accel_y_component_Power_List = [w ** 2 for w in acceleration_y_from_rotational_net_List]

Sum_accel_components_List = list(map(add, Accel_x_component_Power_List, Accel_y_component_Power_List)) 

Accel_xy_resultant_components = np.sqrt(Sum_accel_components_List)

Accel_xy_resultant_components_average = (np.nansum(Accel_xy_resultant_components)) / len(Accel_xy_resultant_components)
Accel_xy_resultant_components_median = np.median(Accel_xy_resultant_components)

#-----------------------------------------------------------------------------------------------------------------
#TASK 6: CALCULATION OF TOTAL FORCE DELIVERED BY THE VEHICLE MOTOR
# Ft = Flinear + Frotational ---> Flinear = Mass * a_linear_x ||| Frotational = Mass * anet ----> Ft = Mass * (a_linear_X + anet)

#Total_Acceleration_Linear_plus_Rotational_List = [sum(k) for k in zip(Acceleration_x_Absolute_List, Anet_List)]
Total_accel_Lin_x = acceleration_x_value_average + acceleration_x_from_rot_anet_average
Total_Accel_Lin_Rot_Value = np.sqrt(Total_accel_Lin_x**2 + acceleration_y_from_rot_anet_average**2) 

#New_Total_Accel_Lin =  np.sqrt((acceleration_x_value_average**2)+(Accel_xy_resultant_components_median**2))


#Ft_List = M * Total_Acceleration_Linear_plus_Rotational_List
#Code above is wrong and returs a huge array = len(Ft_List) = inf

Ft_value = M * Total_accel_Lin_x #New_Total_Accel_Lin #Total_Accel_Lin_Rot_Value


#Ft_List_Absolute = np.abs(Ft_List) ---ALREADY DONE ABS IN PREVIOUS LISTS BEFORE


#-----------------------------------------------------------------------------------------------------------------
#TASK 7: FIND THE TOTAL VELOCITY OF THE VEHICLE = VT = V_linear_X + Vrotational_converted_in_x_y_components

Vel_Linear_x__average_value = average_velocity_x #average_delta_velocity_x

#v_forward = L*(ω/tan α)

V_forward_List_aux =  [l /m for l,m in zip(final_list_delta_angular_velocity_z, tan_yaw_list)]
V_forward_Final_List = [element * L for element in V_forward_List_aux] 
V_forward_dropped_negative_values =  np.array( [num for num in V_forward_Final_List if num>=0])
#check if $ V_Rot_dropped_negative_values is Vrot_Final_List
V_forward_drop_inf =  np.ma.masked_array(V_forward_dropped_negative_values, ~np.isfinite(V_forward_dropped_negative_values)).filled(0)

V_forward_average = np.nansum(V_forward_drop_inf) / len(V_forward_drop_inf)
V_Forward_median = np.median(V_forward_drop_inf)
#Need to convert the Vrot in ax and ay components, using the steer angle, or yaw angle


#The code below is not required anymore, the velocity above is already considered the module of
#x,y speed (Forward)

#auxiliary matrixof cos and sin
Steering_yaw_list_cos_list = np.cos(Steering_yaw_list)
Steering_yaw_list_sin_list = np.sin(Steering_yaw_list)


Vrot_linear_x_component =  [m * n for m,n in zip(V_forward_Final_List, Steering_yaw_list_cos_list)]
Vrot_linear_y_component =  [o * p for o,p in zip(V_forward_Final_List, Steering_yaw_list_sin_list)]

#Removing inf and null values
Vrot_linear_x_component_mask = np.ma.masked_array(Vrot_linear_x_component, ~np.isfinite(Vrot_linear_x_component)).filled(0)
Vrot_linear_y_component_mask = np.ma.masked_array(Vrot_linear_y_component, ~np.isfinite(Vrot_linear_y_component)).filled(0)

#Removing Negative values (opposite velocity from braking)
Vrot_to_linear_x_component_drop_neg =  np.array( [num for num in Vrot_linear_x_component_mask if num>0])
Vrot_to_linear_y_component_drop_neg =  np.array( [num for num in Vrot_linear_y_component_mask if num>0])



#Average of vrot_linear
#Vrot_linear_x_component_average =  (np.nansum(Vrot_to_linear_x_component_drop_neg)) / len(Vrot_to_linear_x_component_drop_neg)
#Vrot_linear_y_component_average = (np.nansum(Vrot_to_linear_y_component_drop_neg)) / len(Vrot_to_linear_y_component_drop_neg)
Vrot_linear_x_component_average = np.median(Vrot_linear_x_component_mask)
Vrot_linear_y_component_average = np.median(Vrot_linear_y_component_mask)



#Need to sum the x components from linear x vel and the rotational converted to linear in x
Total_vel_Linear_x = Vel_Linear_x__average_value + Vrot_linear_x_component_average



#FINAL STEP: Calculate the Total Velocity
#Total Components Values
Power_Total_Linear_Vel_x_value = Total_vel_Linear_x **2
Power_Linear_Vel_y_value = Vrot_linear_y_component_average **2


#Need to sum the vel above 
Total_vel_rot_converted_xy_linear = math.sqrt(Power_Total_Linear_Vel_x_value + Power_Linear_Vel_y_value)


#-----------------------------------------------------------------------------------------------------------------
#TASK 8 CALCULATE THE POWER CONSUMPTION: P = F x V = Ft_List * Velocity_Total_List
Power_Consumption_watts = Ft_value * (V_Forward_median + Vel_Linear_x__average_value )


Power_Consumption_Astar_Planner = Power_Consumption_watts/ 745.699872

#Froim first run Hp = 53 Hp #COOL!!!                                               
#OK THE VALUE IS SUITABLE TO A REAL EXAMPLE CASE: https://www.dummies.com/education/science/physics/how-to-calculate-power-based-on-force-and-speed/
#In the exampple link above the car travels at average 43 km/h during 5 seconds and develop in average 78 hp, a very similar to the value calculate above


#FINALLY THE OPPOSITE FORCES (DRAG AND FRICTION) MUST BE ADDED - SUBTRACTED FROM FIRST CALCULATED FORCE

