#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  6 20:21:19 2021

@author: autoware-auto-ros1
"""


import pandas as pd
import time
#from datetime import date
#import datetime as dt

import datetime as dt

from datetime import datetime

from datetime import timedelta


df = pd.read_csv(r'localizer_pose_op.csv')

print("Original csv: \n")
print(df)
time_stamp_array = df["%time"]
print(time_stamp_array)

#df['%time'] = pd.to_datetime(df[0], unit='ms').apply(lambda x: x.to_datetime64())



unix_t0 = time_stamp_array[0]
unix_t1 = time_stamp_array[1] 
unix_tf = time_stamp_array.iloc[-1]
example = 1360287003083988472


#datetime = dt.datetime.fromtimestamp(unix_t0/10e6)
#datetime2 = dt.datetime.fromtimestamp(unix_t1/10e6)

#datetime = dt.datetime.fromtimestamp(unix_t0).strftime('%Y-%m-%d %H:%M:%S.%f')
#datetime2 = dt.datetime.fromtimestamp(unix_t1).strftime('%Y-%m-%d %H:%M:%S.%f')

#s, ms = divmod(unix_t0, 1000)


#datetime = pd.to_datetime( unix_t0 /1e9, unit='ms')
#datetime2 = pd.to_datetime( unix_t1 /1e9, unit='ms')


time1 = datetime.fromtimestamp(unix_t0 // 1e9)
time2 = datetime.fromtimestamp(unix_t1 // 1e9)


string_dt1 = time1.strftime(('%Y-%m-%d %H:%M:%S')) 
string_dt2 = time2.strftime(('%Y-%m-%d %H:%M:%S'))


str_time1 = string_dt1 + '.' +str(int(unix_t0 % 1000000000)).zfill(9)
str_time1 = str_time1[:-6]


str_time2 = string_dt2 + '.' +str(int(unix_t1 % 1000000000 )).zfill(9)
str_time2 = str_time2[:-6]


t1_float = datetime.strptime(str_time1, '%Y-%m-%d %H:%M:%S.%f')
t2_float = datetime.strptime(str_time2, '%Y-%m-%d %H:%M:%S.%f')

#date_object = datetime.strptime(str_time1, '%Y-%m-%d %H:%M:%S.%f' )



#strip_time = string_datetime.seconds

delta_T = t2_float - t1_float 

print("TEMPO DESEJADO", str_time1)
print("Tempo 2", str_time2)



print("Diferença", delta_T)

#Get the value in miliseconds difference

time_d_ms =  ( (t2_float - t1_float) / dt.timedelta(milliseconds=1) ) *1e-3


#for t in time_stamp_array:
 #   conv_time_msec = datetime.datetime.fromtimestamp(unix_epoch_timestamp_t0/10e6)












'''
T_Initial = time.strftime("%m/%d/%Y %H:%M:%S", time.gmtime(unix_t0/1e9))
#print("Initial time trip", T_Initial)

T_Final = time.strftime("%m/%d/%Y %H:%M:%S", time.gmtime(unix_tf/1e9))
#print("Initial time trip", T_Final)


#Conversion from String to Datetime

T0 = datetime.strptime(T_Initial, '%m/%d/%Y %H:%M:%S')
TF = datetime.strptime(T_Final, '%m/%d/%Y %H:%M:%S')



Trip_Time = TF- T0
Trip_Seconds = Trip_Time.seconds

#print("Total Time Trip", Trip_Time)
#print("Total Time in seconds",Trip_Seconds)
'''