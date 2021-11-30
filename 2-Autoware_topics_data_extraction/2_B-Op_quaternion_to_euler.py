#!/usr/bin/env python
import rospy
#from bagpy import bagreader
import rosbag
import pandas as pd
#check with rosbag info the message type
from geometry_msgs.msg import PoseStamped 
from tf.transformations import euler_from_quaternion
import math

#b = bagreader('open_planner_global_local.bag')
#print(b.topic_table)
bag = rosbag.Bag('open_planner_global_local.bag')
topic_pose = '/current_pose'


#column_names = ['field.pose.orientation.x', 'field.pose.orientation.y', 'field.pose.orientation.z', 'field.pose.orientation.w']
#df = pd.DataFrame(columns=column_names)


yaw_list = []
for topic_pose, msg, t in bag.read_messages(topics=topic_pose):
    
    x = msg.pose.orientation.x
    y = msg.pose.orientation.y
    z = msg.pose.orientation.z
    w = msg.pose.orientation.w
    
    orientation_list = [x,y,z,w]
    (roll,pich,yaw) = euler_from_quaternion(orientation_list)
    yaw_list.append(yaw)

degree_list = []
for degree in range(len(yaw_list)):
    value_degree = math.degrees(yaw_list[degree])
    degree_list.append(value_degree)
    

        

df = pd.DataFrame(list(zip(yaw_list, degree_list)), columns= ['yaw', 'degree'])
df.to_csv('Op_converted_quaternions_to_Euler.csv')
