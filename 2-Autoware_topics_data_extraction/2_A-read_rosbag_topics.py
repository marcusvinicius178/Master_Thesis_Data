#!/usr/bin/env python
import rospy
#from bagpy import bagreader
import rosbag
import pandas as pd
#check with rosbag info the message type
from geometry_msgs.msg import PoseStamped 


#b = bagreader('open_planner_global_local.bag')
#print(b.topic_table)
bag = rosbag.Bag('open_planner_global_local.bag')
topic = '/current_pose'

column_names = ['field.pose.orientation.x', 'field.pose.orientation.y', 'field.pose.orientation.z', 'field.pose.orientation.w']
df = pd.DataFrame(columns=column_names)


for topic, msg, t in bag.read_messages(topics=topic):
    
    x = msg.pose.orientation.x
    y = msg.pose.orientation.y
    z = msg.pose.orientation.z
    w = msg.pose.orientation.w
    
    df = df.append(
        {'field.pose.orientation.x':x, 
         'field.pose.orientation.y':y,
         'field.pose.orientation.z':z,
         'field.pose.orientation.w':w},
        ignore_index=True
        )

df.to_csv('Op_converted_quaternions_to_Euler')



'''
from nav_msgs.msg import Odometry

def get_rotation (msg):
    print msg.pose.pose.orientation

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()
'''
