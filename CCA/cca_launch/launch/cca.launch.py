#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def variable_id(context):
    Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'planner.yaml')
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context)
    params = [Config_file] if vehicle_id == '' else [Config_file,{'vehicle_id' : vehicle_id}]
    Forecast = Node(package='cca_nodes',
             executable='Scout',
             name='Scout',
             parameters=params,
             remappings=[('_action/feedback','Controller_Server/Go_to_waypoint/_action/feedback'),('ovr','resolution')])
    Local_planner = Node(package='cca_nodes',
             executable='LocalPlanner',
             name='Local_planner',
             output='screen',
             parameters=params,
             remappings=[('_action/feedback','Controller_Server/Go_to_waypoint/_action/feedback'),('map','cloud_obstacles')])
    return [Forecast,Local_planner]

def generate_launch_description():
    vehicle_id = LaunchConfiguration('vehicle_id')
    vehicle_id_arg = DeclareLaunchArgument('vehicle_id',default_value='')
    Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'planner.yaml')
    params = [Config_file] if vehicle_id == '' else [Config_file,{'vehicle_id' : vehicle_id}]
    Global_planner = Node(package='cca_nodes',
             executable='GlobalPlanner',
             name='Global_planner',
             parameters=[Config_file],
             remappings=[('map','cloud_obstacles')])
    Path_filter = Node(package='cca_nodes',
             executable='path_filter.py',
             name='Path_filter')
    Controller = Node(package='cca_nodes',
             executable='Conductor.py',
             name='Controller',
             parameters=[Config_file],
             remappings=[('ovr','resolution')])
    ld = LaunchDescription([vehicle_id_arg,Global_planner,Path_filter,Controller])
    ld.add_action(OpaqueFunction(function=variable_id))
    return ld