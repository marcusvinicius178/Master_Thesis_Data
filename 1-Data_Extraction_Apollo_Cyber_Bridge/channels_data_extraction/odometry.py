import os
import errno
import threading 
import csv
import traceback
from cyber.python.cyber_py3 import cyber
# Pose Sensor Libraries
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.localization.proto.localization_pb2 import LocalizationEstimate, LocalizationStatus
#from modules.localization.proto.localization_status_pb2 import LocalizationStatus 
from modules.localization.proto.gps_pb2 import Gps
from modules.drivers.gnss.proto.imu_pb2 import Imu
from modules.drivers.gnss.proto.gnss_best_pose_pb2 import GnssBestPose
from modules.drivers.gnss.proto.ins_pb2 import InsStat
# Obstacles Detection Librarires
from modules.drivers.proto.conti_radar_pb2 import ContiRadar
from modules.drivers.proto.pointcloud_pb2 import PointCloud
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles
from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles
#Navigation Waypoint: Goal data   && Planning Features
from modules.routing.proto.routing_pb2 import RoutingResponse
from modules.planning.proto.planning_pb2 import ADCTrajectory
#Control
from modules.control.proto.control_cmd_pb2 import ControlCommand
#Chassis Interface Data
from modules.canbus.proto.chassis_pb2 import Chassis
#Safety Control
from modules.guardian.proto.guardian_pb2 import GuardianCommand
#Apollo Tasks Manager/Monitor
from modules.monitor.proto.system_status_pb2 import SystemStatus
from modules.common.monitor_log.proto.monitor_log_pb2 import MonitorMessage
from modules.dreamview.proto.hmi_status_pb2 import HMIStatus



user_input = "random_arg"
lattice_directory = '/home/autoware-auto-ros1/ApolloAuto/apollo/cyber/python/cyber_py3/channels_data_extraction/merda'
global my_path
my_path = '/apollo/Traces_csv'

#Data Filename Sense:
global odometry_name 
global corrected_imu_name 
odometry_name = "odometry_messages" 
corrected_imu_name = "corrected_imu_messages"



def define_planner_scenario(*args, **kwargs):

    path_planner = int(input("Entry the Desired Path Planner, Type: 1 [Public_Road], 2 [Lattice] \n\n"))

    if path_planner == 1:
        planner = "Public_Road"
    elif path_planner ==2:
        planner = "Lattice"
    else:
        sys.exit(1)


    scenario = int(input("Entry the case, Type: 1 [Pedestrian], 2 [Overtake], 3 [Turn],  4 [Stop_and_Go] or 5 [Static_Truck] \n\n"))

    if scenario == 1:
        case = "Pedestrian"
    elif scenario ==2:
        case = "Overtake"
    elif scenario == 3:
        case = "Turn"
    elif scenario == 4:
        case = "Stop_and_Go"
    elif scenario == 5:
        case = "Static_Truck"
    else:
        sys.exit(1)

    print("Selected Path Planner: ", planner)
    print("Selected Case-Scenario: ", case)

    print("The csv will be soon produced in the related folders")

    global prefix
    global suffix
    prefix = planner
    suffix = case  


    global cyber_channels_data_flow_path_dir
    cyber_channels_data_flow_path_dir = os.path.join(my_path, prefix+"_"+suffix)

    
    
def create_specific_folder(path):
    try:
        os.makedirs(path)
        path = os.makedirs(path)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
            raise
        pass
    return path
    print("The path is", path)


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
    odometry_file_name = prefix+"_"+odometry_name+"_"+suffix+".csv"
    odometry_final_name = os.path.join(cyber_channels_data_flow_path_dir, odometry_file_name)

    try:
        csv_is_empty = check_csv_file_empty(odometry_final_name)
        with open(odometry_final_name, mode='a') as csv_file:
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



def corrected_imu_message_parser_callback(data):
    global corrected_imu_file_name
    corrected_imu_file_name = prefix+"_"+corrected_imu_name+"_"+suffix+".csv"
    corrected_imu_final_name = os.path.join(cyber_channels_data_flow_path_dir, corrected_imu_file_name)

    try:
        csv_is_empty = check_csv_file_empty(corrected_imu_final_name)
                
        with open(corrected_imu_final_name, mode='a') as csv_file:
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



if __name__ == '__main__':
     
    cwd = os.getcwd()
    print("Current working directory is:", cwd)
    #create_specific_folder('/apollo/Lattice_Traces/OI')
    define_planner_scenario(user_input)
    create_specific_folder(cyber_channels_data_flow_path_dir)

    cyber.init()
    message_parser_node = cyber.Node("message_parser")
    message_parser_node.create_reader("/apollo/sensor/gnss/odometry", Gps, odometry_message_parser_callback)
    message_parser_node.create_reader("/apollo/sensor/gnss/corrected_imu", CorrectedImu, corrected_imu_message_parser_callback)
        


    message_parser_node.spin()

    cyber.shutdown()
