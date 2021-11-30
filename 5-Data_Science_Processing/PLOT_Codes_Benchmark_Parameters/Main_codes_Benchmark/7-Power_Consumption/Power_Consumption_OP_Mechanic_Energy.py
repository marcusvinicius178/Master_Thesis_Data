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
import matplotlib.pyplot as plt
from scipy import stats
from sympy import *
import sympy as sp
from statistics import stdev, variance
import regex as re

df_vel_1 = pd.read_csv(r'Op_current_pose_velocity.csv')
df_pose_1 = pd.read_csv(r'Op_current_pose_velocity.csv')

df_yaw_angle_aux = pd.read_csv(r'Op_converted_quaternions_to_Euler.csv')


df_synchronize_yaw_to_current_pose_velocity = df_pose_1.merge(df_yaw_angle_aux, on="field.header.seq", copy=False)
df_synchronize_yaw_to_current_pose_velocity.to_csv('complete_df.csv')


last_trajectory_waypoint = -35.33454132080078 #From Astar_df.csv
#The last waypoint of the smallest goal point was selected (From Astar = -35 in x.pose)
filter_last_waypoint_standardize_planners = df_synchronize_yaw_to_current_pose_velocity[df_synchronize_yaw_to_current_pose_velocity['field.pose.position.x']>=last_trajectory_waypoint]
df_difference = filter_last_waypoint_standardize_planners['field.pose.position.x'].diff()
df_abs_difference = df_difference.abs()
#Removing the time of modules activation in Autoware.AI Runtime Manager
filtered_real_car_journey = filter_last_waypoint_standardize_planners[df_abs_difference>0.05]

#Outliers_fst_quartile = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]] < (Q1 - 1.5 * IQR) 


df_yaw_angle = filtered_real_car_journey[['yaw', 'degree']]
df_vel = filtered_real_car_journey[['%time_x', 'field.twist.linear.x', 'field.twist.linear.y','field.twist.angular.z']]
Op_df_vel = df_vel
df_pose = filtered_real_car_journey[['%time_x','field.pose.position.x', 'field.pose.position.y']]





#From manual, still not from LGSVL Unity 3D, just to test
global L
L = float(2.834891)   #float(2.72796) #wheelbase in meters of Jaguar Xe model
M = float(2120) # Kg taken in LGSVL support - unix measurement

#Check if the csv Counters-Timestamp from files above are aligned, if not, drop the lines manually to synchronize
# and then replace "," to "." because after drop operation it loses the format...and do not work the code..



#Get Time_Stamp columns in UInix Epoch (Machine Format)
time_stamp_array = df_pose["%time_x"]
#Get the column of velocity data in x(m/s)
vel_linear_x_array = df_vel["field.twist.linear.x"]
vel_linear_y_array = df_vel['field.twist.linear.y']
#Get the columns of angular velocity in z(rad/s)
vel_angular_z_array = df_vel["field.twist.angular.z"]
#There is NOT Linear velocity in Y, because the car has nonholomic constraint, that's why need to convert Z in x and y velocity later
orientation_yaw_array = df_yaw_angle["yaw"]
#Pose array 
pose_x_array = df_pose["field.pose.position.x"]
pose_y_array = df_pose["field.pose.position.y"]
#To use to plot the Obstacle distance
pose_x_array_Op = pose_x_array
pose_y_array_Op = pose_y_array


#TASKS OF POWER CALCULATION TUTORIAL

#TASK 1: CONVERT TIME MACHINE TO HUMAN AND GET VELOCITY VARIATION AND ACCELERATION IN X AXIS

# Loop inside the Unix Epoch TimeStamp List To convert to Humna Readable Format
#Time List Extraction to Human Format

final_list = []
my_list = []
for i in range(len(time_stamp_array)):
    
    time_value = time_stamp_array.iloc[i]
    human_readable = datetime.fromtimestamp(time_value // 1e9)
    string_human_readable = human_readable.strftime(('%Y-%m-%d %H:%M:%S')) 
    
    str_human_time = string_human_readable + '.' +str(int(time_value % 1000000000)).zfill(9)
    #required to drop the last 6 characters related to microseconds otherwise next line will not process it
    str_human_time = str_human_time[:-6]
    

    human_time_float = datetime.strptime(str_human_time, '%Y-%m-%d %H:%M:%S.%f')
    #print(human_time_float)
    final_list.append(human_time_float)


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
Op_cum_time = np.cumsum(progressive_time_list)

#-----------------------TRIP TOTAL TIME CALCULATION------------------------------------------------
unix_epoch_timestamp_t0 = time_stamp_array.iloc[0]
unix_epoch_timestamp_tf = time_stamp_array.iloc[-1]
T_Initial = time.strftime("%m/%d/%Y %H:%M:%S", time.gmtime(unix_epoch_timestamp_t0/1e9))
T_Final = time.strftime("%m/%d/%Y %H:%M:%S", time.gmtime(unix_epoch_timestamp_tf/1e9))
#Conversion from String to Datetime
T0 = datetime.strptime(T_Initial, '%m/%d/%Y %H:%M:%S')
TF = datetime.strptime(T_Final, '%m/%d/%Y %H:%M:%S')
Trip_Time = TF- T0
Trip_Seconds_Op_1 = Trip_Time.seconds
Trip_seconds_Op_2 = sum(final_list_delta_seconds) #This is more exact!!! 
Trip_seconds_Op = Trip_seconds_Op_2

Op_trip_seconds = np.linspace(0,Trip_seconds_Op,len(time_stamp_array))

#-------------------------------------------------------------------------------------------------
    
    
#Calculate the Delta velocity of Time n - Time(n-1)
final_list_delta_velocity_x = []
for v_x in range(len(vel_linear_x_array)-1):
    #v_x_value = vel_linear_x_array[v_x]
    #print(v_x_value)
    delta_v_x = vel_linear_x_array.iloc[v_x+1] - vel_linear_x_array.iloc[v_x]
    final_list_delta_velocity_x.append(delta_v_x)
        
        
#Extract the Acceleration  = Velocity_linear_x_Array / Human Radable Time in Seconds 
acceleration_x_list = [a /b for a,b in zip(final_list_delta_velocity_x, final_list_delta_seconds)]
#Negative values mean that the car is braking, therefore does not mean the car is generating power in these 
#times and will prejudice the average value calculation pulling to small value
drop_brake_opposite_accel_values_List = np.array( [num for num in acceleration_x_list if num>=0])
drop_brake_opposite_velocity_list = np.array( [num for num in final_list_delta_velocity_x if num>=0])


#The acceleration just in x, if I do the loop for acceleration y list, the velocity y=0...will result in empty list
Op_accel_xy = acceleration_x_list
Op_accel_xy.insert(0,0) #The first velocity  value there is not acceleration, so a=0
#------------------------------------------------------------------------------------------------------------------

#TASK2: GET THE ANGULAR VELOCITY VARIATION = Δω = Rad/s
#CCW is positive and CW is negative - However it is not required if condition, as the difference will always show if the move is mainly CCW or CW

final_list_delta_angular_velocity_z = []
for v_z in range(len(vel_angular_z_array)-1):
    angular_velocity = vel_angular_z_array.iloc[v_z]
    delta_v_z = vel_angular_z_array.iloc[v_z+1] - vel_angular_z_array.iloc[v_z]
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
    steer_yaw_value = orientation_yaw_array.iloc[steer]
    delta_steer_value = steer_yaw_value - orientation_yaw_array.iloc[0] #because the first value is starting with negative value (-3 = -178 deg) and next is (-2 turning CCW) = the difference is 1 rad, so -2 - (-3) = -2 +3 = 1 rad
    Steering_yaw_list.append(delta_steer_value) #The same here as angular velocity, does not matter if the car rotated CCW or CW to calculate the power...


Op_Steering_yaw_List = Steering_yaw_list
Op_Steering_yaw_List.insert(0,0)

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


#-------------------------------------------------------------------------------------------------
#TASK 6 Find the Delta Pose in x,y 

final_list_delta_pose_x = []
for p_x in range(len(pose_x_array)-1):
    
    delta_pose_x_value = abs(pose_x_array.iloc[p_x+1]) - abs(pose_x_array.iloc[p_x])
    final_list_delta_pose_x.append(delta_pose_x_value)

delta_x_pose_abs_list = [abs(element) for element in final_list_delta_pose_x]
Cum_delta_x_pose_abs_list_Op = sum(delta_x_pose_abs_list)

length = len(delta_x_pose_abs_list)
Cummulative_delta_x_pose_list = [sum(delta_x_pose_abs_list[0:x:1]) for x in range(0, length+1)]
Cummulative_delta_x_pose_list.pop(0)

Op_cummulative_xy_position = Cummulative_delta_x_pose_list
origin = 0
Op_cum_xy_position = [origin] + Op_cummulative_xy_position


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

#CORRIGIR O DELTA para xi,y1
delta_y_pose_abs_list = [abs(element) for element in final_list_delta_pose_y]

standard_x_goal_waypoint_planners = round(abs(pose_x_array.iloc[-1]-pose_x_array.iloc[0])/10)*10
Total_path_x_traveled_Op = sum(delta_x_pose_abs_list) #standard_x_goal_waypoint_planners #
Total_path_y_traveled_Op = sum(delta_y_pose_abs_list)
Total_path_xy_traveled_Op = math.sqrt(Total_path_x_traveled_Op**2 + Total_path_y_traveled_Op**2)

Cummulative_pose_xy_Op = delta_pose_xy_module_list.sum()


Filter_by_cum_x_Df_Op = pd.DataFrame({'Cummulative X': Cummulative_delta_x_pose_list,
                                   'Delta X':delta_x_pose_abs_list,
                                   'DElta Y':delta_y_pose_abs_list})

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

remove_Outliers_df_z_Score_method = Joined_accel_pose_df[(z < 30).all(axis=1)] #try 40 to eliminate remove of outlier
#Changing the z value (reducing will improve eliminate outliers points)

#Using IQR 
Q1 = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]].quantile(0.25)
Q3 = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]].quantile(0.75)
IQR = Q3 - Q1
#print(IQR)
Outliers_fst_quartile = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]] < (Q1 - 1.5 * IQR) 
Outliers_4th_quartile = Joined_accel_pose_df[["Acceleration X", "Acceleration Y"]] > (Q3 + 1.5 * IQR)

remove_Outliers_df_IQR_method = Joined_accel_pose_df[~((Joined_accel_pose_df < (Q1 - 1.5 * IQR)) |(Joined_accel_pose_df > (Q3 + 1.5 * IQR))).any(axis=1)]

Filtered_Parameters_Df = remove_Outliers_df_IQR_method #remove_Outliers_df_z_Score_method 
#The IQR method is not very suitable for my case, removing redundand waypoints.
#Without remove the outliers it also returns a reasonable value...if remove the data where the vehicle is stopped.

#------------------------------------------------------------------------------------------------
#After Data Analysis and remotion above, returning to Calculation


#Acceleration List = Acceleration X = Positive

Acceleration_List = Filtered_Parameters_Df #[(Filtered_Parameters_Df[['Acceleration X']] > 0).all(1)]

Acceleration_xy_df = Acceleration_List[['Acceleration X', 'Acceleration Y']]


Acceleration_xy_Pow_df = Acceleration_xy_df.pow(2)
Sum_Acceleration_xy_pow_df = Acceleration_xy_Pow_df['Acceleration X'] + Acceleration_xy_Pow_df['Acceleration Y']
Resultant_Acceleration_xy_Sqrt_List = Sum_Acceleration_xy_pow_df**(1/2)
Resultant_accel_df = pd.DataFrame({'Acceleration XY':Resultant_Acceleration_xy_Sqrt_List})
Acceleration_rejoin_delta_df = pd.merge(Acceleration_List[['Delta Pose XY', 'Delta Time']], Resultant_accel_df, left_index=True, right_index=True)
Acceleration_xy_df_Op = Acceleration_rejoin_delta_df #To use in Plot_traj_smotheness module.py


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

#Mean/Average/Median Acceleration Power (HP)
Mean_Op_Acceleration_power = Power_Acceleration_df_hp.mean()
Median_Op_Acceleration_power = Power_Acceleration_df_hp.median()
#deviation
std_dev_Op_acceleration = stdev(Power_Acceleration_df_hp)
variance_Op_acceleration = variance(Power_Acceleration_df_hp)
#CV variation coefficient = Standard deviation / mean. If CV < 1 indicates a low variation, however for non-normal distribution the standard deviation does not mean anything, also the CV
#Source: https://www.researchgate.net/post/What-do-you-consider-a-good-standard-deviation
CV_Accel_Power = std_dev_Op_acceleration / Mean_Op_Acceleration_power


#BHP is not the energy spend while the vehicle brakes, it is just a form or energy spent measurementin laboratory.

#FRICTION POWER 

#Mean/Average?/Median Brake Power (BHP)
Mean_Op_brake_power = Power_Brake_df_hp.mean()
Median_Op_brake_power = Power_Brake_df_hp.median()

std_dev_Op_brake = stdev(Power_Brake_df_hp)
variance_Op_acceleration = variance(Power_Brake_df_hp)
#CV variation coefficient = Standard deviation / mean. If CV < 1 indicates a low variation, however for non-normal distribution the standard deviation does not mean anything, also the CV
#It just says how much your data is spread from the mean...but does not say if it is wrong!
CV_Brake_Power = std_dev_Op_brake / Mean_Op_brake_power


pd.set_option("display.max_rows", None, "display.max_columns", None)


#Ok Power =12 Hp the value obtained is pretty similar to the second mode below. It just differs a little bit 
#because the outliers from acceleration differs from the outliers from velocitu


#-----------------------------ATTEMPT2------------Using KINETIC ENERGY VARIATION = M*V² - fit curve - derivative

V_linear_delta_x = final_list_delta_velocity_x

#Converting the rotational velocity to linear

V_forward_List_aux =  [l /m for l,m in zip(final_list_delta_angular_velocity_z, tan_yaw_list)]
V_forward_Final_List = [element * L for element in V_forward_List_aux] 

#auxiliary matrixof cos and sin
Steering_yaw_list_cos_list = np.cos(Steering_yaw_list)
Steering_yaw_list_sin_list = np.sin(Steering_yaw_list)


Vrot_linear_x_component =  [m * n for m,n in zip(V_forward_Final_List, Steering_yaw_list_cos_list)]
Vrot_linear_y_component =  [o * p for o,p in zip(V_forward_Final_List, Steering_yaw_list_sin_list)]


Total_linear_x_velocity = list(map(add, V_linear_delta_x, Vrot_linear_x_component ))
Total_linear_y_velocity = Vrot_linear_y_component



#Convert the velocities above in DataFrame to split in Brake and Acceleration move.

Joined_velocity_xy_df = pd.DataFrame({'Velocity X':Total_linear_x_velocity,
                                     'Velocity Y':Total_linear_x_velocity,
                                     'Delta Time': final_list_delta_seconds})


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


Positive_Velocity_df = Filtered_Parameters_velocity_Df[(Filtered_Parameters_velocity_Df[['Velocity X']] > 0).all(1)]
Negative_Velocity_df = Filtered_Parameters_velocity_Df[(Filtered_Parameters_velocity_Df[['Velocity X']] < 0).all(1)]


Total_positive_velocity_xy_df = Positive_Velocity_df[['Velocity X', 'Velocity Y']]
Positive_velocity_xy_Pow_df = Total_positive_velocity_xy_df.pow(2)
Sum_Positive_velocity_xy_pow_df = Positive_velocity_xy_Pow_df['Velocity X'] + Positive_velocity_xy_Pow_df['Velocity Y']
Resultant_Positive_Velocity_xy_Sqrt_df = Sum_Positive_velocity_xy_pow_df**(1/2)


list_aux_delta_time = Positive_Velocity_df['Delta Time'].to_list()
list_aux_res_vel = Resultant_Positive_Velocity_xy_Sqrt_df.to_list()

Delta_Time_df = pd.DataFrame({"Delta Time":list_aux_delta_time})
New_Resultant_Positive_Velocity_xy_Sqrt_df = pd.DataFrame({"Resultant Velocity":list_aux_res_vel})


get_back_complete_df = pd.concat([New_Resultant_Positive_Velocity_xy_Sqrt_df, Delta_Time_df], axis=1)



#Filter Again the Outliers
Q1_res_v = get_back_complete_df[["Resultant Velocity", "Delta Time"]].quantile(0.25)
Q3_res_v = get_back_complete_df[["Resultant Velocity", "Delta Time"]].quantile(0.75)
IQR_vres = Q3_res_v - Q1_res_v
#print(IQR)
Outliers_fst_quartile_Q1_res = get_back_complete_df[["Resultant Velocity", "Delta Time"]] < (Q1 - 1.5 * IQR_vres) 
Outliers_4th_quartile_Q1_res = get_back_complete_df[["Resultant Velocity", "Delta Time"]] > (Q3 + 1.5 * IQR_vres)

remove_Outliers_velocity_res_df_IQR_method = get_back_complete_df[~((get_back_complete_df < (Q1_res_v - 1.5 * IQR_vres)) |(get_back_complete_df > (Q3_res_v + 1.5 * IQR_vres))).any(axis=1)]

Filtered_Parameters_velocity_res_v_Df = remove_Outliers_velocity_res_df_IQR_method


'''
#Divide ΔV/Δt
Aux_Dataframe_vel_per_time = Filtered_Parameters_velocity_res_v_Df["Resultant Velocity"] / Filtered_Parameters_velocity_res_v_Df["Delta Time"]
Aux_df_to_list = Aux_Dataframe_vel_per_time.to_list()
Aux_Dataframe_vel_per_time = pd.DataFrame({"Energy Power":Aux_df_to_list})


#Filtering Outliers

Q1_energy = Aux_Dataframe_vel_per_time[["Energy Power"]].quantile(0.25)
Q3_energy = Aux_Dataframe_vel_per_time[["Energy Power"]].quantile(0.75)
IQR_energy = Q3_energy - Q1_energy
#print(IQR)
Outliers_fst_quartile_Q1_energy = Aux_Dataframe_vel_per_time[["Energy Power"]] < (Q1 - 1.5 * IQR_energy) 
Outliers_4th_quartile_Q1_energy = Aux_Dataframe_vel_per_time[["Energy Power"]] > (Q3 + 1.5 * IQR_energy)

remove_Outliers_velocity_energy_IQR_method = Aux_Dataframe_vel_per_time[~((Aux_Dataframe_vel_per_time < (Q1_energy - 1.5 * IQR_energy)) |(Aux_Dataframe_vel_per_time > (Q3_energy + 1.5 * IQR_energy))).any(axis=1)]

Filtered_Parameters_velocity_energy_Df = remove_Outliers_velocity_energy_IQR_method



#Calculate Mechanic Energy Variation / Time  = ΔE = mVf² - mV0² = m(ΔV)/ Time = Power in Watts for each instant/waypoint
Delta_kinetic_mec_energy_array_per_time = M * Filtered_Parameters_velocity_energy_Df
Power_Consumption_Horsepower_df = Delta_kinetic_mec_energy_array_per_time / 745.699872


#Calculate Power Diving ΔE/Δt
#Power_Consumption_Watts_df = Delta_kinetic_mec_energy_array / Positive_Velocity_df["Delta Time"]
#Power_Consumption_Horsepower_df = Power_Consumption_Watts_df / 745.699872
'''

#calculate Mechanica Energy Variation  ΔE = mVf² - mV0² 

delta_mechanic_energy = (M * New_Resultant_Positive_Velocity_xy_Sqrt_df)/2
delta_mechanic_list_aux = delta_mechanic_energy["Resultant Velocity"].to_list()
delta_mechanic_column_rename = pd.DataFrame({'Mechanic Energy (ΔE)': delta_mechanic_list_aux})

Power_Consumption_Watts_df = delta_mechanic_column_rename["Mechanic Energy (ΔE)"] / Delta_Time_df["Delta Time"]
Power_Consumption_Horsepower_df = Power_Consumption_Watts_df / 745.699872

Delta_cumulative_time = Delta_Time_df["Delta Time"].cumsum()
aux_delta_time_list = Delta_cumulative_time.to_list()
New_Delta_cumulative_time = pd.DataFrame({'Cummulative Time Variation (ΔT)':aux_delta_time_list})
plot_df = pd.concat([delta_mechanic_column_rename, New_Delta_cumulative_time], axis=1)

#Plot To find the function which is suitable to curve

x = New_Delta_cumulative_time["Cummulative Time Variation (ΔT)"]                   
y = delta_mechanic_column_rename["Mechanic Energy (ΔE)"] 
#The first number is infinite and is crashing the lineregress function below
x.pop(0)
y.pop(0)


ax1 = plot_df.plot.scatter(x="Cummulative Time Variation (ΔT)", y="Mechanic Energy (ΔE)") #Actually is mechanic energy

m,b = np.polyfit(x,y,1)
plt.plot(x, m*x + b)


slope, intercept, r_value, p_value, std_err = stats.linregress(x,y)
res= stats.linregress(x,y)
plt.plot(x, y, 'o', label='original data')

X = symbols('x')
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


Instantaneous_Power_Watts_Df =  first_coef_round + (-last_coef_of_x_round *Delta_Time_df["Delta Time"])
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

