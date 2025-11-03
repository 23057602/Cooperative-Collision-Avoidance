#!/usr/bin/env python3
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	Perception = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cca_launch'),'launch', 'dep.launch.py')])
     )
	PlanningControl = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cca_launch'),'launch', 'cca.launch.py')])
     )
	Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'planner.yaml')
	tf_app = Node(package='cca_launch',
             executable='TfAsOdom.py',
             name='pose_node',
             parameters=[Config_file],
             remappings=[('odom','odometry/filtered'),('tf_odom','pose')])
	Head_node = Node(package='cca_launch',
             executable='Voyager.py',
             name='Voyager_command',
             parameters=[Config_file])
	return LaunchDescription([Perception, PlanningControl,tf_app,Head_node])