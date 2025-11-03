#!/usr/bin/env python3
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	EKF_Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'ekf.yaml')
	OdometryEKF = Node(package='robot_localization', 
					executable='ekf_node', 
					name='ekf_filter_node', 
					parameters=[EKF_Config_file])
	RTAB_Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'rtab.yaml')
	Slam = Node(package='rtabmap_slam', 
					executable='rtabmap', 
					name='slam', 
					parameters=[RTAB_Config_file],
					remappings=[('imu','/imu_plugin/out'),('scan_cloud','/ouster/points')])
	Bridge_Config_file = os.path.join(get_package_share_directory('cca_launch'),'config', 'bridge.yaml')
	Bridge = Node(package='domain_bridge', 
					executable='domain_bridge', 
					name='com_bridge', 
					arguments=[Bridge_Config_file])
	return LaunchDescription([OdometryEKF, Slam, Bridge])