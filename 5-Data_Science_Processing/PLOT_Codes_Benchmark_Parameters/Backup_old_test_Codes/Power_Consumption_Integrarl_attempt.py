#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 09:30:44 2021

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

df_vel = pd.read_csv(r'current_velocity_op.csv')
df_pose = pd.read_csv(r'current_pose_op.csv')
df_yaw_angle = pd.read_csv(r'Op_converted_quaternions_to_Euler.csv')

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
#Pose array 
pose_x_array = df_pose["field.pose.position.x"]
pose_y_array = df_pose["field.pose.position.y"]



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
    

progressive_time_list = list(map(sum,zip(final_list_delta_seconds, [0]+final_list_delta_seconds)))

    
    
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

acceleration_y_from_rotational_net_List = []

for accel_y_rot in range(len(Theta_decomposition_angle_List)):
    accel_y_component_value = Anet_List[accel_y_rot] * (math.cos(Theta_decomposition_angle_List[accel_y_rot]))
    acceleration_y_from_rotational_net_List.append(accel_y_component_value)

drop_negative_opposite_accel_y = np.array( [num for num in acceleration_y_from_rotational_net_List if num>=0])


Total_acceleration_x_List = [m + n for m,n in zip(acceleration_x_list, acceleration_x_from_rotational_net_List)]
Total_acceleration_y_List = acceleration_y_from_rotational_net_List


'''
percentile_list = pd.DataFrame(
    {'lst1Title': lst1,
     'lst2Title': lst2,
     'lst3Title': lst3
    })
'''
#-------------------------------------------------------------------------------------------------
#TASK 6 Find the Delta Pose in x,y 

final_list_delta_pose_x = []
for p_x in range(len(pose_x_array)-1):
    
    delta_pose_x_value = pose_x_array[p_x+1] - pose_x_array[p_x]
    final_list_delta_pose_x.append(delta_pose_x_value)


final_list_delta_pose_y = []
for p_y in range(len(pose_y_array)-1):
    
    delta_pose_y_value = pose_y_array[p_y+1] - pose_y_array[p_y]
    final_list_delta_pose_y.append(delta_pose_y_value)


delta_pose_x_pow = [x **2 for x in final_list_delta_pose_x]
delta_pose_y_pow = [y **2 for y in final_list_delta_pose_y]

Sum_pose_components_List = list(map(add, delta_pose_x_pow, delta_pose_y_pow))

delta_pose_xy_module_list = np.sqrt(Sum_pose_components_List)


#-------------------------------------------------------------------------------------------------
#TASK 7 
#Join the Acceleration x, acceleration y and delta pose list, in order to separate the positive values (Power Hp from accel) and negative values (brake)
Joined_accel_pose_df = pd.DataFrame({'Acceleration X':Total_acceleration_x_List,
                                     'Acceleration Y':Total_acceleration_y_List,
                                     'Delta Pose XY': delta_pose_xy_module_list,
                                     'Delta Time': final_list_delta_seconds})


#--------------------------------------------------------------------------------------------------
#Task 8: REMOVING THE OUTLIERS: Before keeping the calculation of Force and Power it is requiered
#Identifyng the Outliers to remove
sns.boxplot(Joined_accel_pose_df["Acceleration X"])
sns.boxplot(Joined_accel_pose_df["Acceleration Y"])

#Plotting this graph it is possible to visualize a lot of points far from the quartiles that must
#be removed because they are increasing the Power to a value too high.
#Source: https://towardsdatascience.com/ways-to-detect-and-remove-the-outliers-404d16608dba

#USING Z SCORE METHOD
z = np.abs(stats.zscore(Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]]))
threshold = 3
#print(np.where(z > 3))

remove_Outliers_df_z_Score_method = Joined_accel_pose_df[(z < 3).all(axis=1)]
#Changing the z value (reducing will improve eliminate outliers points)

#Using IQR 
Q1 = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]].quantile(0.25)
Q3 = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]].quantile(0.75)
IQR = Q3 - Q1
#print(IQR)
Outliers_fst_quartile = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]] < (Q1 - 1.5 * IQR) 
Outliers_4th_quartile = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]] > (Q3 + 1.5 * IQR)

remove_Outliers_df_IQR_method = Joined_accel_pose_df[~((Joined_accel_pose_df < (Q1 - 1.5 * IQR)) |(Joined_accel_pose_df > (Q3 + 1.5 * IQR))).any(axis=1)]

Filtered_Parameters_Df = remove_Outliers_df_IQR_method

#------------------------------------------------------------------------------------------------
#After Data Analysis and remotion above, returning to Calculation


#Acceleration List = Acceleration X = Positive

Acceleration_List = Filtered_Parameters_Df[(Filtered_Parameters_Df[['Acceleration X']] > 0).all(1)]

Acceleration_xy_df = Acceleration_List[['Acceleration X', 'Acceleration Y']]
Acceleration_xy_Pow_df = Acceleration_xy_df.pow(2)
Sum_Acceleration_xy_pow_df = Acceleration_xy_Pow_df['Acceleration X'] + Acceleration_xy_Pow_df['Acceleration Y']
Resultant_Acceleration_xy_Sqrt_List = Sum_Acceleration_xy_pow_df**(1/2)


#Brake List = Acceleration X = Negative

Brake_List = Filtered_Parameters_Df[(Filtered_Parameters_Df[['Acceleration X']] < 0).all(1)]

Brake_xy_df = Brake_List[['Acceleration X', 'Acceleration Y']]
Brake_xy_Pow_df = Brake_xy_df.pow(2)
Sum_Brake_xy_pow_df = Brake_xy_Pow_df['Acceleration X'] + Brake_xy_Pow_df['Acceleration Y']
Resultant_Brake_xy_Sqrt_List = Sum_Brake_xy_pow_df**(1/2)


#-----------------------------------------------------------------------------------------------------------------
#TASK 8: CALCULATION OF TOTAL FORCE DELIVERED BY THE VEHICLE MOTOR
#Separate opposite force (brake) from positive (accel)
# F = M * a
Force_Acceleration_df = Resultant_Acceleration_xy_Sqrt_List.apply(lambda x: x*M)
Force_Brake_df = Resultant_Brake_xy_Sqrt_List.apply(lambda x: x*M)


#TASk 9: Calcualte instantaneous Action= work = N.m = Joule

Action_Acceleration_df = Force_Acceleration_df * Acceleration_List['Delta Pose XY'] 
Action_Brake_df = Force_Brake_df * Brake_List['Delta Pose XY'] 



#Taks 10: Calculate Power in Hp

Power_Acceleration_df_watts = Action_Acceleration_df / Acceleration_List['Delta Time']
Power_Brake_df_watts = Action_Brake_df / Brake_List['Delta Time']

Power_Acceleration_df_hp = Power_Acceleration_df_watts.apply(lambda x: x/745.699872)
Power_Brake_df_hp = Power_Brake_df_watts.apply(lambda x: x/745.699872)

Total_Power_Acceleration_df_Hp = Power_Acceleration_df_hp.sum()
Total_Brake_df_hp = Power_Brake_df_hp.sum()


Average_Total_Power_Acceleration_df_Hp = Total_Power_Acceleration_df_Hp / len(Power_Acceleration_df_hp)
Average_Total_Brake_df_hp = Total_Brake_df_hp / len(Power_Brake_df_watts)

#Mean = Average
Mean_Acceleration_power = Power_Acceleration_df_hp.mean()
Median_Acceleration_power = Power_Acceleration_df_hp.median()

pd.set_option("display.max_rows", None, "display.max_columns", None)

