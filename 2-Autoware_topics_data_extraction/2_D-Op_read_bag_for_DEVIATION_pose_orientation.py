#!/usr/bin/env python
import rospy
#from bagpy import bagreader
import rosbag
import pandas as pd
#check with rosbag info the message type
from geometry_msgs.msg import PoseStamped 
from tf.transformations import euler_from_quaternion
import math
from Op_read_bag_for_DEVIATION_pose_orientation import *

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
    
timestamp_list = []
seq_list = []

for topic_pose, msg, t in bag.read_messages(topics=topic_pose):
    timestamp = t
    timestamp_list.append(timestamp)
    seq = msg.header.seq
    seq_list.append(seq)    
    

pose_position_x = []
pose_position_y = []    
for topic_pose, msg, t in bag.read_messages(topics=topic_pose):

    x_value = msg.pose.position.x
    pose_position_x.append(x_value)
    y_value = msg.pose.position.y
    pose_position_y.append(y_value)
    



df = pd.DataFrame(list(zip(timestamp_list, seq_list, yaw_list, degree_list, pose_position_x, pose_position_y, )), columns= ['%time','field.header.seq', 'yaw', 'degree', 'field.pose.position.x','field.pose.position.y'])
df.to_csv('Op_current_pose_for_Deviation_calc.csv')
