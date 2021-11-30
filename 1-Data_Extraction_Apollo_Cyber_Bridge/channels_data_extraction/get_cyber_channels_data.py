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
global my_path
my_path = '/apollo/Traces_csv'

#Data Filename Sense/Localization module:
global odometry_name 
odometry_name = "odometry_messages" 
global corrected_imu_name 
corrected_imu_name = "corrected_imu_messages"
global raw_imu_name
raw_imu_name = "raw_imu_message"
global localization_name
localization_name = "localization"
global localization_status_name
localization_status_name = "localization_status"

#Data Filenames from Drivers/ Perception module:
global gnss_best_pose_name
gnss_best_pose_name = "gnss_best_pose"
global radar_name
radar_name = "conti_radar"
global lidar_name
lidar_name = "velodyne_lidar"
global pointcloud_name
pointcloud_name = "pointcloud"
global ins_name
ins_name = "ins"

#Data Filenames: Prediction Module
global traffic_light_name
traffic_light_name = "traffic_light"
global prediction_obstacle_name
prediction_obstacle_name = "prediction_obstacle"

#Data Filenames Navigation/Planning modules:
global routing_name
routing_name = "routing"
global planning_matrix_info_name
planning_matrix_info_name = "Planning_matrix_info"
global planning_name 
planning_name = "planning"


#Data Filenames Control/Chassis module:
global control_name
control_name = "control"
global chassis_name
chassis_name = "chassis"

#Data Filenama Security Module:
global guardian_name
guardian_name = "guardian"

#Data Systems/Monitor/User Interfaces:
global system_status_name
system_status_name = "system_status"
global monitor_name
monitor_name = "monitor"
global dreamview_hmi_name
dreamview_hmi_name = "dreamview_hmi"



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
        n = str(1)
        case = "Pedestrian"
    elif scenario ==2:
        n = str(2)
        case = "Overtake"
    elif scenario == 3:
        n = str(3)
        case = "Turn"
    elif scenario == 4:
        n = str(4)
        case = "Stop_and_Go"
    elif scenario == 5:
        n = str(5)
        case = "Static_Truck"
    else:
        sys.exit(1)

    print("Selected Path Planner: ", planner)
    print("Selected Case-Scenario: ", case)

    print("The csv will be soon produced in the related folders")

    global prefix
    global suffix
    global Case_N
    Case_N = n
    prefix = planner
    suffix = case  


    global cyber_channels_data_flow_path_dir
    cyber_channels_data_flow_path_dir = os.path.join(my_path, Case_N+"-"+prefix+"_"+suffix)

    
    
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


def raw_imu_parser_callback(data):
    global raw_imu_file_name
    raw_imu_file_name = prefix+"_"+raw_imu_name+"_"+suffix+".csv"
    raw_imu_final_name = os.path.join(cyber_channels_data_flow_path_dir, raw_imu_file_name)

    try:
        csv_is_empty = check_csv_file_empty(raw_imu_final_name)

        with open(raw_imu_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp','LinearAcceleration','LinearAcceleration_x','LinearAcceleration_y','LinearAcceleration_z',
                          'AngularVelocity', 'AngularVelocity_x','AngularVelocity_y','AngularVelocity_z']   


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'LinearAcceleration': '',
                            'LinearAcceleration_x':data.linear_acceleration.x,
                            'LinearAcceleration_y':data.linear_acceleration.y,
                            'LinearAcceleration_z':data.linear_acceleration.z,
                            'AngularVelocity': '',
                            'AngularVelocity_x':data.angular_velocity.x,
                            'AngularVelocity_y':data.angular_velocity.y,
                            'AngularVelocity_z':data.angular_velocity.z})

    except:
        print(traceback.format_exc())


def localization_parser_callback(data):
    global localization_file_name
    localization_file_name = prefix+"_"+localization_name+"_"+suffix+".csv"
    localization_final_name = os.path.join(cyber_channels_data_flow_path_dir, localization_file_name)
    try:
        csv_is_empty = check_csv_file_empty(localization_final_name)

        with open(localization_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp','Pose', 'Position_x', 'Position_y', 'Position_z',
            'Orientation', 'Qx', 'Qy', 'Qz', 'Qw',
            'LinearVelocity', 'LinearVelocity_x', 'LinearVelocity_y', 'LinearVelocity_z',
            'LinearAcceleration', 'LinearAcceleration_x', 'LinearAcceleration_y', 'LinearAcceleration_z',
            'AngularVelocity', 'AngularVelocity_x','AngularVelocity_y','AngularVelocity_z', 'Heading',
            'LinearAccelerationVRF', 'LinearAccelerationVRF_x','LinearAccelerationVRF_y','LinearAccelerationVRF_z',
            'AngularVelocityVRF', 'AngularVelocityVRF_x','AngularVelocityVRF_y', 'AngularVelocityVRF_z',
            'EulerAngles', 'EulerAngles_x','EulerAngles_x','EulerAngles_y','EulerAngles_z', 'Measurement_Time'] 


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec, 'Pose':'', 'Position_x':data.pose.position.x,'Position_y':data.pose.position.y, 'Position_z':data.pose.position.z,
                'Orientation':'','Qx':data.pose.orientation.qx, 'Qy':data.pose.orientation.qy, 'Qz':data.pose.orientation.qz, 'Qw':data.pose.orientation.qw,
                'LinearVelocity':'', 'LinearVelocity':'', 'LinearVelocity_x':data.pose.linear_velocity.x, 'LinearVelocity_y':data.pose.linear_velocity.y, 'LinearVelocity_z':data.pose.linear_velocity.z,
                'LinearAcceleration':'', 'LinearAcceleration_x':data.pose.linear_acceleration.x, 'LinearAcceleration_y':data.pose.linear_acceleration.y, 'LinearAcceleration_z':data.pose.linear_acceleration.z,
                'AngularVelocity':'', 'AngularVelocity_x':data.pose.angular_velocity.x, 'AngularVelocity_y':data.pose.angular_velocity.y, 'AngularVelocity_z':data.pose.angular_velocity.z, 'Heading':data.pose.heading,
                'LinearAccelerationVRF':'', 'LinearAccelerationVRF_x':data.pose.linear_acceleration_vrf.x, 'LinearAccelerationVRF_y':data.pose.linear_acceleration_vrf.y, 'LinearAccelerationVRF_z':data.pose.linear_acceleration_vrf.z,
                'AngularVelocityVRF':'', 'AngularVelocityVRF_x':data.pose.angular_velocity_vrf.x, 'AngularVelocityVRF_y':data.pose.angular_velocity_vrf.y, 'AngularVelocityVRF_z':data.pose.angular_velocity_vrf.z, 
                'EulerAngles':'', 'EulerAngles_x':data.pose.euler_angles.x, 'EulerAngles_y':data.pose.euler_angles.y, 'EulerAngles_z':data.pose.euler_angles.z,
                'Measurement_Time':data.measurement_time })

    except:
        print(traceback.format_exc())



def localization_status_parser_callback(data):
    global localization_status_file_name
    localization_status_file_name = prefix+"_"+localization_status_name+"_"+suffix+".csv"
    localization_status_final_name = os.path.join(cyber_channels_data_flow_path_dir, localization_status_file_name)

    try:
        csv_is_empty = check_csv_file_empty(localization_status_final_name)

        with open(localization_status_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Measurement_Time', 'Fusion_Status', 'State_Message']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec, 'Measurement_Time': data.measurement_time,
                'Fusion_Status':data.fusion_status,'State_Message':data.state_message})

    except:
        print(traceback.format_exc())



def ins_parser_callback(data):
    global ins_file_name
    ins_file_name = prefix+"_"+ins_name+"_"+suffix+".csv"
    ins_final_name = os.path.join(cyber_channels_data_flow_path_dir, ins_file_name)
    try:
        csv_is_empty = check_csv_file_empty(ins_final_name)

        with open(ins_file_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'INS_Status', 'Pos_Type']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,'INS_Status':data.ins_status, 'Pos_Type':data.pos_type})

    except:
        print(traceback.format_exc())



def radar_parser_callback(data):
    global radar_file_name
    radar_file_name = prefix+"_"+radar_name+"_"+suffix+".csv"
    radar_final_name = os.path.join(cyber_channels_data_flow_path_dir, radar_file_name)
    try:
        csv_is_empty = check_csv_file_empty(radar_final_name)

        with open(radar_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Obstacle_Array_Info','Object_List_Status', 'N_of_Objects', 'Measurement_Counter']   


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Obstacle_Array_Info':data.contiobs,
                            'Object_List_Status':'',
                            'N_of_Objects': data.object_list_status.nof_objects,
                            'Measurement_Counter':data.object_list_status.meas_counter})

    except:
        print(traceback.format_exc())


def lidar_parser_callback(data):
    global lidar_file_name
    lidar_file_name = prefix+"_"+lidar_name+"_"+suffix+".csv"
    lidar_final_name = os.path.join(cyber_channels_data_flow_path_dir, lidar_file_name)
    try:
        csv_is_empty = check_csv_file_empty(lidar_final_name)

        with open(lidar_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Perception_Obstacles_Matrix']   


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Perception_Obstacles_Matrix':data.perception_obstacle})

    except:
        print(traceback.format_exc())


def point_cloud_parser_callback(data):
    global pointcloud_file_name
    pointcloud_file_name = prefix+"_"+pointcloud_name+"_"+suffix+".csv"
    pointcloud_final_name = os.path.join(cyber_channels_data_flow_path_dir, pointcloud_file_name)
    try:
        csv_is_empty = check_csv_file_empty(pointcloud_final_name)

        with open(pointcloud_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Lidar_TimeStamp', 'Is_Dense', 'PointCloud_Matrix', 'Measurement_Time', 'Width', 'Height']   


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Lidar_TimeStamp':data.header.lidar_timestamp,'Is_Dense':data.is_dense,'PointCloud_Matrix':data.point,'Measurement_Time':data.measurement_time,
                            'Width':data.width, 'Height':data.height })

    except:
        print(traceback.format_exc())


def traffic_light_parser_callback(data):
    global traffic_light_file_name
    traffic_light_file_name = prefix+"_"+traffic_light_name+"_"+suffix+".csv"
    traffic_light_final_name = os.path.join(cyber_channels_data_flow_path_dir, traffic_light_file_name)
    try:
        csv_is_empty = check_csv_file_empty(traffic_light_final_name)

        with open(traffic_light_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Contain_Lights']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Contain_Lights':data.contain_lights})

    except:
        print(traceback.format_exc())


def prediction_parser_callback(data):
    global prediction_obstacle_file_name
    prediction_obstacle_file_name = prefix+"_"+prediction_obstacle_name+"_"+suffix+".csv"
    prediction_obstacle_final_name = os.path.join(cyber_channels_data_flow_path_dir, prediction_obstacle_file_name)
    try:
        csv_is_empty = check_csv_file_empty(prediction_obstacle_final_name)

        with open(prediction_obstacle_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Obstacle_Action_Prediction']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Obstacle_Action_Prediction':data.prediction_obstacle})

    except:
        print(traceback.format_exc())


def routing_parser_callback(data):
    global routing_file_name
    routing_file_name = prefix+"_"+routing_name+"_"+suffix+".csv"
    routing_final_name = os.path.join(cyber_channels_data_flow_path_dir, routing_file_name)
    try:
        csv_is_empty = check_csv_file_empty(routing_final_name)

        with open(routing_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Road_Info', 'Goal_Distance', 'N_of_Waypoints']  


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Road_Info':data.road,
                            'Goal_Distance':data.measurement.distance,
                            'N_of_Waypoints':data.routing_request.waypoint})

    except:
        print(traceback.format_exc())



def planning_matrix_info_parser_callback(data):
    global planning_matrix_info_filename
    planning_matrix_info_filename = prefix+"_"+planning_matrix_info_name+"_"+suffix+".csv"
    planning_matrix_info_final_name= os.path.join(cyber_channels_data_flow_path_dir, planning_matrix_info_filename)
    try:
        csv_is_empty = check_csv_file_empty(planning_matrix_info_final_name)

        with open(planning_matrix_info_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Path_Matrix_Info', 'Path_point_x', 'Path_point_y', 'Path_info_theta',
            'Path_info_kappa', 'Path_dist_modulus', 'Path_info_deviation_kappa']   


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                'Path_Matrix_Info':data.debug.planning_data.path})

    except:
        print(traceback.format_exc())





def planning_parser_callback(data):
    global planning_file_name
    planning_file_name = prefix+"_"+planning_name+"_"+suffix+".csv"
    planning_final_name = os.path.join(cyber_channels_data_flow_path_dir, planning_file_name)
    try:
        csv_is_empty = check_csv_file_empty(planning_final_name)

        with open(planning_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Total_Path_Length', 'Total_Path_Time',
            'Path_Matrix_Info', 'ADC_Spot_General', 'Pose', "Position_x", 'Position_y', 'Position_z',
            'Orientation', 'Qx' , 'Qy', 'Qz', 'Qw', 
            'LinearVelocity', 'LinearVelocity_x', 'LinearVelocity_y', 'LinearVelocity_z',
            'LinearAcceleration', 'LinearAcceleration_x', 'LinearAcceleration_y', 'LinearAcceleration_z',
            'AngularVelocity', 'AngularVelocity_x', 'AngularVelocity_y', 'AngularVelocity_z', 'Heading',
            'LinearAccelerationVRF', 'LinearAccelerationVRF_x', 'LinearAccelerationVRF_y', 'LinearAccelerationVRF_z',
            'AngularVelocityVRF', 'AngularVelocityVRF_x', 'AngularVelocityVRF_y', 'AngularVelocityVRF_z',
            'EulerAngles', 'EulerAngles_x', 'EulerAngles_y', 'EulerAngles_z',
            'Measurement_Time',
            'Chassis', 'Engine_Status_Started', 'Engine_RPM', 'Speed_mps', 'Odometer_m', 'Fuel_Range_M', 'Throttle_Percentage', 'Brake_Percentage',
            'Steering_Percentage', 'Parking_Brake', 'Wiper', 'Driving_Mode', 'Gear_Location']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec, 'Total_Path_Length': data.total_path_length, 'Total_Path_Time': data.total_path_time,
                'Path_Matrix_Info':'', 'ADC_Spot_General':'', 
                'Pose':'', 'Position_x':data.debug.planning_data.adc_position.pose.position.x, 'Position_y':data.debug.planning_data.adc_position.pose.position.y,
                'Position_z':data.debug.planning_data.adc_position.pose.position.z, 'Orientation':'', 'Qx':data.debug.planning_data.adc_position.pose.orientation.qx, 'Qy':data.debug.planning_data.adc_position.pose.orientation.qy, 'Qz':data.debug.planning_data.adc_position.pose.orientation.qz, 'Qw':data.debug.planning_data.adc_position.pose.orientation.qw,
                'LinearVelocity':'', 'LinearVelocity_x':data.debug.planning_data.adc_position.pose.linear_velocity.x, 'LinearVelocity_y':data.debug.planning_data.adc_position.pose.linear_velocity.y, 'LinearVelocity_z':data.debug.planning_data.adc_position.pose.linear_velocity.z,
                'LinearAcceleration': '', 'LinearAcceleration_x':data.debug.planning_data.adc_position.pose.linear_acceleration.x, 'LinearAcceleration_y':data.debug.planning_data.adc_position.pose.linear_acceleration.y, 'LinearAcceleration_z':data.debug.planning_data.adc_position.pose.linear_acceleration.z,
                'AngularVelocity':'', 'AngularVelocity_x':data.debug.planning_data.adc_position.pose.angular_velocity.x, 'AngularVelocity_y':data.debug.planning_data.adc_position.pose.angular_velocity.y, 'AngularVelocity_z':data.debug.planning_data.adc_position.pose.angular_velocity.z, 'Heading':data.debug.planning_data.adc_position.pose.heading,
                'LinearAccelerationVRF': '', 'LinearAccelerationVRF_x':data.debug.planning_data.adc_position.pose.linear_acceleration_vrf.x, 'LinearAccelerationVRF_y':data.debug.planning_data.adc_position.pose.linear_acceleration_vrf.y, 'LinearAccelerationVRF_z':data.debug.planning_data.adc_position.pose.linear_acceleration_vrf.z,
                'AngularVelocityVRF':'', 'AngularVelocityVRF_x': data.debug.planning_data.adc_position.pose.angular_velocity_vrf.x, 'AngularVelocityVRF_y': data.debug.planning_data.adc_position.pose.angular_velocity_vrf.y, 'AngularVelocityVRF_z': data.debug.planning_data.adc_position.pose.angular_velocity_vrf.z,
                'EulerAngles':'', 'EulerAngles_x':data.debug.planning_data.adc_position.pose.euler_angles.x, 'EulerAngles_y':data.debug.planning_data.adc_position.pose.euler_angles.y, 'EulerAngles_z':data.debug.planning_data.adc_position.pose.euler_angles.z,
                'Measurement_Time':data.debug.planning_data.adc_position.measurement_time,
                'Chassis':data.debug.planning_data.chassis, 
                'Engine_Status_Started':data.debug.planning_data.chassis.engine_started, 'Engine_RPM':data.debug.planning_data.chassis.engine_rpm,
                'Speed_mps':data.debug.planning_data.chassis.speed_mps,
                "Odometer_m":data.debug.planning_data.chassis.odometer_m, 'Fuel_Range_M':data.debug.planning_data.chassis.fuel_range_m,
                'Throttle_Percentage':data.debug.planning_data.chassis.throttle_percentage,'Brake_Percentage':data.debug.planning_data.chassis.brake_percentage,
                'Steering_Percentage':data.debug.planning_data.chassis.steering_percentage, 'Parking_Brake':data.debug.planning_data.chassis.parking_brake, 'Wiper':data.debug.planning_data.chassis.wiper,
                'Driving_Mode':data.debug.planning_data.chassis.driving_mode, 'Gear_Location':data.debug.planning_data.chassis.gear_location })


    except:
        print(traceback.format_exc())

#Space for control/chassis/etc test in paperspace stronger machine...

def control_parser_callback(data):
    global control_file_name
    control_file_name = prefix+"_"+control_name+"_"+suffix+".csv"
    control_final_name = os.path.join(cyber_channels_data_flow_path_dir, control_file_name)

    try:
        csv_is_empty = check_csv_file_empty(control_final_name)

        with open(control_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Throttle', 'Brake', 'Steering_Rate', 'Steering_Target', 'Acceleration', 'Gear_Location',
            'Debug','Station_Reference', 'Station_Error', 'Station_Error_Limited', 'Preview_Station_Error', 'Speed_Reference', 'Speed_Error',
            'Speed_Controller_Input_Limited', 'Preview_Speed_Reference', 'Preview_Speed_Error', 'Preview_Acceleration_Reference', 'Acceleration_Cmd_Closeloop',
            'Acceleration_Cmd','Acceleration_Lookup','Speed_Lookup', 'Calibration_Value', 'Throttle_Cmd', 'Brake_Cmd', 'Is_Full_Stop', 'Slope_Offset_Compensation', 
            'Current_Station', 'Path_Remain', 'PID_Saturation_Status', 'Speed_Offset', 'Current_Speed', 'Acceleration_Reference', 'Current_Acceleration', 'Acceleration_Error',
            'Jerk_Reference', 'Current_Jerk', 'Jerk_Error',
            'Current_Matched_Point', 'Path_Point','Path_Point_x', 'Path_Point_y',
            'Current_Reference_Point', 'Path_Point_Ref', 'Path_Point_Ref_x', 'Path_Point_Ref_y',
            'Preview_Reference_Point', 'Path_Point_Prev', 'Path_Point_Prev_x', 'Path_Point_Prev_y',
            'Simple_Lat_Debug', 'Lateral_Error', 'Ref_Heading', 'Heading', 'Heading_Error', 'Heading_Error_Rate','Lateral_Error_Rate', 'Curvature' ]    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec, 'Throttle': data.throttle, 'Brake': data.brake, 'Steering_Rate': data.steering_rate,
                            'Steering_Target':data.steering_target, 'Acceleration':data.acceleration, 'Gear_Location':data.gear_location,
                            'Debug':'', 'Station_Reference':data.debug.simple_lon_debug.station_reference, 'Station_Error':data.debug.simple_lon_debug.station_error, 'Station_Error_Limited':data.debug.simple_lon_debug.station_error_limited,
                            'Preview_Station_Error':data.debug.simple_lon_debug.preview_station_error, 'Speed_Reference':data.debug.simple_lon_debug.speed_reference, 'Speed_Error':data.debug.simple_lon_debug.speed_error, 'Speed_Controller_Input_Limited':data.debug.simple_lon_debug.speed_controller_input_limited,
                            'Preview_Speed_Reference':data.debug.simple_lon_debug.preview_speed_reference, 'Preview_Speed_Error':data.debug.simple_lon_debug.preview_speed_error, 'Preview_Acceleration_Reference':data.debug.simple_lon_debug.preview_acceleration_reference,
                            'Acceleration_Cmd_Closeloop':data.debug.simple_lon_debug.acceleration_cmd_closeloop, 'Acceleration_Cmd':data.debug.simple_lon_debug.acceleration_cmd, 'Acceleration_Lookup':data.debug.simple_lon_debug.acceleration_lookup, 'Speed_Lookup':data.debug.simple_lon_debug.speed_lookup,
                            'Calibration_Value':data.debug.simple_lon_debug.calibration_value, 'Throttle_Cmd':data.debug.simple_lon_debug.throttle_cmd, 'Brake_Cmd':data.debug.simple_lon_debug.brake_cmd, 'Is_Full_Stop':data.debug.simple_lon_debug.is_full_stop, 'Slope_Offset_Compensation':data.debug.simple_lon_debug.slope_offset_compensation,
                            'Current_Station':data.debug.simple_lon_debug.current_station, 'Path_Remain':data.debug.simple_lon_debug.path_remain, 'PID_Saturation_Status':data.debug.simple_lon_debug.pid_saturation_status, 'Speed_Offset':data.debug.simple_lon_debug.speed_offset, 'Current_Speed':data.debug.simple_lon_debug.current_speed, 'Acceleration_Reference':data.debug.simple_lon_debug.acceleration_reference,
                            'Current_Acceleration':data.debug.simple_lon_debug.current_acceleration, 'Acceleration_Error':data.debug.simple_lon_debug.acceleration_error,'Jerk_Reference':data.debug.simple_lon_debug.jerk_reference, 'Current_Jerk':data.debug.simple_lon_debug.current_jerk,'Jerk_Error':data.debug.simple_lon_debug.jerk_error,
                            'Current_Matched_Point':'', 'Path_Point':'', 'Path_Point_x':data.debug.simple_lon_debug.current_matched_point.path_point.x, 'Path_Point_y':data.debug.simple_lon_debug.current_matched_point.path_point.y,
                            'Current_Reference_Point':'', 'Path_Point_Ref':'', 'Path_Point_Ref_x':data.debug.simple_lon_debug.current_reference_point.path_point.x, 'Path_Point_Ref_y':data.debug.simple_lon_debug.current_reference_point.path_point.y,
                            'Preview_Reference_Point':'', 'Path_Point_Prev':'', 'Path_Point_Prev_x':data.debug.simple_lon_debug.preview_reference_point.path_point.x, 'Path_Point_Prev_y':data.debug.simple_lon_debug.preview_reference_point.path_point.y,
                            'Simple_Lat_Debug':'', 'Lateral_Error':data.debug.simple_lat_debug.lateral_error, 'Ref_Heading':data.debug.simple_lat_debug.ref_heading, 'Heading':data.debug.simple_lat_debug.heading, 'Heading_Error':data.debug.simple_lat_debug.heading_error,
                            'Heading_Error_Rate':data.debug.simple_lat_debug.heading_error_rate, 'Lateral_Error_Rate':data.debug.simple_lat_debug.lateral_error_rate, 'Curvature':data.debug.simple_lat_debug.curvature })

    except:
        print(traceback.format_exc())



def chassis_parser_callback(data):
    global chassis_file_name
    chassis_file_name = prefix+"_"+chassis_name+"_"+suffix+".csv"
    chassis_final_name = os.path.join(cyber_channels_data_flow_path_dir, chassis_file_name)
    try:
        csv_is_empty = check_csv_file_empty(chassis_final_name)

        with open(chassis_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp','Engine_Started', 'Engine_RPM', 'Speed_mps', 'Odometer_m', 'Fuel_Range_M', 'Throttle_Percentage', 'Brake_Percentage', 
            'Steering_Percentage', 'Parking_Brake', 'Wiper', 'Driving_Mode', 'Gear_Location', 
            'Chassis_GPS', 'Latitude','Longitude','GPS_Valid','Compass_Direction', 'Pdop', 'Is_Gps_Fault', 'Is_Inferred', 'Altitude', 'Heading', 'Hdop', 'Vdop',
            'Quality', 'Num_of_Satellites', 'GPS_Speed']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Engine_Started':data.engine_started, 'Engine_RPM':data.engine_rpm, 'Speed_mps':data.speed_mps,
                            'Odometer_m':data.odometer_m, 'Fuel_Range_M':data.fuel_range_m, 'Throttle_Percentage':data.throttle_percentage, 'Brake_Percentage':data.brake_percentage,
                            'Steering_Percentage':data.steering_percentage, 'Parking_Brake':data.parking_brake, 'Wiper':data.wiper, 'Driving_Mode': data.driving_mode, 'Gear_Location':data.gear_location,
                            'Chassis_GPS':'','Latitude':data.chassis_gps.latitude, 'Longitude':data.chassis_gps.longitude, 'GPS_Valid':data.chassis_gps.gps_valid, 'Compass_Direction':data.chassis_gps.compass_direction,
                            'Pdop':data.chassis_gps.pdop,'Is_Gps_Fault':data.chassis_gps.is_gps_fault,'Is_Inferred':data.chassis_gps.is_inferred,'Altitude':data.chassis_gps.altitude,'Heading':data.chassis_gps.heading,
                            'Altitude':data.chassis_gps.altitude,'Heading':data.chassis_gps.heading,'Hdop':data.chassis_gps.hdop,'Vdop':data.chassis_gps.vdop,
                            'Num_of_Satellites':data.chassis_gps.num_satellites,'GPS_Speed':data.chassis_gps.gps_speed})

    except:
        print(traceback.format_exc())

    


def guardian_parser_callback(data):
    global guardian_file_name
    guardian_file_name = prefix+"_"+guardian_name+"_"+suffix+".csv"
    guardian_final_name = os.path.join(cyber_channels_data_flow_path_dir, guardian_file_name)

    try:
        csv_is_empty = check_csv_file_empty(guardian_final_name)

        with open(guardian_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Control_Cmd', 'Throttle', 'Brake', 'Steering_Rate',
            'Steering_Target', 'Is_In_Safe_Mode' ]  


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec, 'Control_Cmd':'',
            'Throttle':data.control_command.throttle, 'Brake': data.control_command.brake,
            'Steering_Rate':data.control_command.steering_rate, 'Steering_Target':data.control_command.steering_target,
            'Is_In_Safe_Mode':data.control_command.is_in_safe_mode})

    except:
        print(traceback.format_exc())



def system_status_parser_callback(data):
    global system_status_file_name
    system_status_file_name = prefix+"_"+system_status_name+"_"+suffix+".csv"
    system_status_final_name = os.path.join(cyber_channels_data_flow_path_dir, system_status_file_name)
    try:
        csv_is_empty = check_csv_file_empty(system_status_final_name)

        with open(system_status_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Module_Name', 'Passenger_Msg', 'Safety_Mode_Trigger_Time',
            'Require_Emergency_stop', 'HMI_Modules_Matrix', 'Components_Matrix']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,'Module_Name':data.header.module_name,
                'Passenger_Msg':data.passenger_msg, 'Safety_Mode_Trigger_Time': data.safety_mode_trigger_time,
                'Require_Emergency_stop':data.require_emergency_stop, 'HMI_Modules_Matrix':data.hmi_modules,
                'Components_Matrix':data.components
                            })

    except:
        print(traceback.format_exc())



def monitor_parser_callback(data):
    global monitor_file_name
    monitor_file_name = prefix+"_"+monitor_name+"_"+suffix+".csv"
    monitor_final_name = os.path.join(cyber_channels_data_flow_path_dir, monitor_file_name)
    try:
        csv_is_empty = check_csv_file_empty(monitor_final_name)

        with open(monitor_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Module_Name', 'Item_Matrix']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Module_Name':data.header.module_name, 'Item_Matrix':data.item})

    except:
        print(traceback.format_exc())


def dreamview_hmi_parser_callback(data):
    global dreamview_hmi_file_name
    dreamview_hmi_file_name = prefix+"_"+dreamview_hmi_name+"_"+suffix+".csv"
    dreamview_hmi_final_name = os.path.join(cyber_channels_data_flow_path_dir, dreamview_hmi_file_name)
    try:
        csv_is_empty = check_csv_file_empty(dreamview_hmi_final_name)

        with open(dreamview_hmi_final_name, mode='a') as csv_file:
            fieldnames = ['TimeStamp', 'Module_Name', 'Modes', 'Current_Mode', 'Maps', 'Current_Map',
            'Vehicles', 'Current_Vehicle', 'Modules', 'Monitored_Components', 'UTM_Zone_Id']    


            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            if(csv_is_empty):
                writer.writeheader()
                csv_is_empty = False

            writer.writerow({'TimeStamp': data.header.timestamp_sec,
                            'Module_Name':data.header.module_name, 'Modes': data.modes, 'Current_Mode':data.current_mode,
                            'Vehicles':data.vehicles, 'Current_Vehicle':data.current_vehicle, 'Modules':data.modules, 
                            'Monitored_Components':data.monitored_components, 'UTM_Zone_Id':data.utm_zone_id})

    except:
        print(traceback.format_exc())



if __name__ == '__main__':
     
    cwd = os.getcwd()
    print("Current working directory is:", cwd)
    #create_specific_folder('/apollo/Lattice_Traces/OI')
    define_planner_scenario(user_input)
    create_specific_folder(cyber_channels_data_flow_path_dir)


    cyber.init()
# Sensor data channels    
    message_parser_node = cyber.Node("message_parser")
    message_parser_node.create_reader("/apollo/sensor/gnss/odometry", Gps, odometry_message_parser_callback)
    message_parser_node.create_reader("/apollo/sensor/gnss/corrected_imu", CorrectedImu, corrected_imu_message_parser_callback)
    message_parser_node.create_reader("/apollo/sensor/gnss/imu", Imu,raw_imu_parser_callback)
    message_parser_node.create_reader("/apollo/localization/pose", LocalizationEstimate, localization_parser_callback)
    message_parser_node.create_reader("/apollo/localization/msf_status", LocalizationStatus, localization_status_parser_callback)
# Drivers Perception channels
    message_parser_node.create_reader("/apollo/sensor/conti_radar", ContiRadar, radar_parser_callback)
    message_parser_node.create_reader("/apollo/sensor/lidar128/compensator/PointCloud2", PointCloud, point_cloud_parser_callback)
    message_parser_node.create_reader("/apollo/perception/obstacles", PerceptionObstacles, lidar_parser_callback)
    message_parser_node.create_reader("/apollo/sensor/gnss/ins_stat", InsStat, ins_parser_callback)
# Data Obstacles Prediction
    message_parser_node.create_reader("/apollo/perception/traffic_light", TrafficLightDetection, traffic_light_parser_callback)
    message_parser_node.create_reader("/apollo/prediction", PredictionObstacles, prediction_parser_callback)
#Navigation Goal Data
    message_parser_node.create_reader("/apollo/routing_response", RoutingResponse, routing_parser_callback)
    message_parser_node.create_reader("/apollo/planning", ADCTrajectory, planning_parser_callback)
    #There is an issue getting the data below...see how to get with DataFrame or other format instead csv...or let be the last data to get.
    #message_parser_node.create_reader("/apollo/planning", ADCTrajectory, planning_matrix_info_parser_callback)
#Control Data
    message_parser_node.create_reader("/apollo/control", ControlCommand, control_parser_callback)
#Chassis Data
    message_parser_node.create_reader("/apollo/canbus/chassis", Chassis, chassis_parser_callback)
#Safety Control
    message_parser_node.create_reader("/apollo/guardian", GuardianCommand, guardian_parser_callback)
#Apollo System Manager
    message_parser_node.create_reader("/apollo/monitor/system_status", SystemStatus, system_status_parser_callback)
    message_parser_node.create_reader("/apollo/monitor", MonitorMessage, monitor_parser_callback)
    message_parser_node.create_reader("/apollo/hmi/status", HMIStatus, dreamview_hmi_parser_callback )



    message_parser_node.spin()

    cyber.shutdown()
