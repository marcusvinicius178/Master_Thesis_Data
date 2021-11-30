#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
"""Module for example of listener."""

from cyber.python.cyber_py3 import cyber
from modules.localization.proto.gps_pb2 import Gps
import os
import errno

lattice_directory = '/home/autoware-auto-ros1/ApolloAuto/apollo/cyber/python/cyber_py3/channels_data_extraction/coco3'
#odometry_file_path = 
odometry_file_name = "odometry_messages.csv"


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
    #global lattice_directory
    global odometry_file_name
    try:
        #create_specific_folder(path = lattice_directory)
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


if __name__ == '__main__':
    
    create_specific_folder(lattice_directory)
    
    #cyber.init()
    #message_parser_node = cyber.Node("message_parser")
    #message_parser_node.create_reader("/apollo/sensor/gnss/odometry", Gps, odometry_message_parser_callback)
    #message_parser_node.spin()

    cyber.shutdown()