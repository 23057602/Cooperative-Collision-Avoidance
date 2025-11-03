#!/usr/bin/env python3
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	vehicle_id = LaunchConfiguration('vehicle_id')
	vehicle_id_arg = DeclareLaunchArgument('vehicle_id',default_value='')
	PlanningControl = IncludeLaunchDescription(
        	PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cca_launch'),'launch', 'cca.launch.py')]),
			launch_arguments={'vehicle_id':vehicle_id}.items())
	Bridge_Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'bridge.yaml')
	net_args = [Bridge_Config_file] if os.getenv('ROS_DOMAIN_ID') == None else ['--from', os.getenv('ROS_DOMAIN_ID'), Bridge_Config_file]
	Bridge = Node(package='domain_bridge', 
					executable='domain_bridge', 
					name='com_bridge', 
					arguments=net_args)
	Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'planner.yaml')
	Head_node = Node(package='cca_launch',
             executable='VirtualVoyager.py',
             name='Virtual_command',
             parameters=[Config_file])
	return LaunchDescription([vehicle_id_arg,PlanningControl,Bridge,Head_node])