#!/usr/bin/env python3
import threading 
import csv
import traceback
from cyber.python.cyber_py3 import cyber
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.localization.proto.gps_pb2 import Gps
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
import os


odometry_file_name = "odometry_messages.csv"
imu_file_name = "imu_message.csv"
gnss_best_pose_file_name = "gnss_best_pose.csv"

CASO = odometry.callback.case

def check_csv_file_empty(a_file):
	try:
		csv_is_empty = True
		with open(a_file, mode='r') as csv_file:
			csv_dict = [row for row in csv.DictReader(csv_file)]
			if(len(csv_dict) > 0):
				return False
	except:
		return True
		
def odometry_message_parser_callback(data):
	global odometry_file_name
	try:
		csv_is_empty = check_csv_file_empty(odometry_file_name)
		with open(odometry_file_name, mode='a') as csv_file:
			fieldnames = ['TimeStamp','SequenceNumber','Position','Position_x', 'Position_y','Position_z',
						  'Orientation', 'Orientation_x','Orientation_y','Orientation_z', 'Orientation_w',
						  'LinearVelocity', 'LinearVelocity_x','LinearVelocit_y','LinearVelocity_z']
						  
			writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
			
			if(csv_is_empty):
				writer.writeheader()
				csv_is_empty = False
				
			writer.writerow({'TimeStamp': data.header.timestamp_sec,
							 'SequenceNumber': data.header.sequence_num,
							'Position': '',
							'Position_x': data.localization.position.x,
							'Position_y': data.localization.position.y,
							'Position_z': data.localization.position.z,
							'Orientation': '',
							'Orientation_x': data.localization.orientation.qx,
							'Orientation_y': data.localization.orientation.qy,
							'Orientation_z': data.localization.orientation.qz,
							'Orientation_w': data.localization.orientation.qw,
							'LinearVelocity':'',
							'LinearVelocity_x': data.localization.linear_velocity.x,
							'LinearVelocit_y': data.localization.linear_velocity.y,
							'LinearVelocity_z': data.localization.linear_velocity.z})	
		
	except:
		print(traceback.format_exc())

		
def imu_message_parser_callback(data):
	global imu_file_name
	try:
		csv_is_empty = check_csv_file_empty(imu_file_name)
				
		with open(imu_file_name, mode='a') as csv_file:
			fieldnames = ['TimeStamp','LinearAcceleration','LinearAcceleration_x','LinearAcceleration_y', 'LinearAcceleration_z',
						  'AngularVelocity', 'AngularVelocity_x','AngularVelocity_y','AngularVelocity_z',
						  'Heading', 'EulerAngles', 'EulerAngles_x','EulerAngles_y','EulerAngles_z']
						  
			writer = csv.DictWriter(csv_file,fieldnames=fieldnames)
			
			if(csv_is_empty):
				writer.writeheader()
				csv_is_empty = False
				
			writer.writerow({'TimeStamp': data.header.timestamp_sec,
							'LinearAcceleration': '',
							'LinearAcceleration_x': data.imu.linear_acceleration.x,
							'LinearAcceleration_y': data.imu.linear_acceleration.y,
							'LinearAcceleration_z': data.imu.linear_acceleration.z,
							'AngularVelocity': '',
							'AngularVelocity_x': data.imu.angular_velocity.x,
							'AngularVelocity_y': data.imu.angular_velocity.y,
							'AngularVelocity_z': data.imu.angular_velocity.z,
							'Heading': data.imu.heading, 
							'EulerAngles':'',
							'EulerAngles_x': data.imu.euler_angles.x,
							'EulerAngles_y': data.imu.euler_angles.y,
							'EulerAngles_z': data.imu.euler_angles.z})	
		
	except:
		print(traceback.format_exc())

def gnss_best_pose_parser_callback(data):
	global gnss_best_pose_file_name
	try:
		csv_is_empty = check_csv_file_empty(gnss_best_pose_file_name)

		with open(gnss_best_pose_file_name, mode='a') as csv_file:
			fieldnames = ['TimeStamp','Latitude', 'Longitude']	


			writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

			if(csv_is_empty):
				writer.writeheader()
				csv_is_empty = False

			writer.writerow({'TimeStamp': data.header.timestamp_sec,
							'Latitude':data.latitude,
							'Longitude':data.longitude})

	except:
		print(traceback.format_exc())
	
if __name__=='__main__':
	os.chdir(r"/home/autoware-auto-ros1/ApolloAuto/apollo/Lattice_Planner_Traces")
	cwd = os.getcwd()
	print("Current working directory is:", cwd)
	print("Caso é", case)
	cyber.init()
	message_parser_node = cyber.Node("message_parser")

	message_parser_node.create_reader("/apollo/sensor/gnss/corrected_imu", CorrectedImu, imu_message_parser_callback)
	message_parser_node.create_reader("/apollo/sensor/gnss/odometry", Gps, odometry_message_parser_callback)
	message_parser_node.create_reader("/apollo/sensor/gnss/best_pose", GnssBestPose, gnss_best_pose_parser_callback)
	
	message_parser_node.spin()

	cyber.shutdown()
	
