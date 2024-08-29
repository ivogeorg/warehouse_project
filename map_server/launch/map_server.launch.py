#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    map_svr_pkg_name = 'map_server'
    config_dir_name = 'config'
    default_map_file_name = 'warehouse_map_sim.yaml'

    # Capture command-line or launch-file argument 'map_file'
    #
    # Note: 
    # This is only the file name and not the full path, so
    # construct the path and then pass to the 'yaml_filename'
    # parameter of the 'map_server' executable.
    map_file_arg = DeclareLaunchArgument(
                        'map_file', 
                        default_value=TextSubstitution(text=default_map_file_name))
    map_file_f = LaunchConfiguration('map_file')
    map_file_path = PathJoinSubstitution([FindPackageShare(map_svr_pkg_name), config_dir_name, map_file_f])

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path}])

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    rviz_config_file_name = 'config.rviz'
    rviz_config_dir_name = 'rviz'
    rviz_config_dir = PathJoinSubstitution([FindPackageShare(map_svr_pkg_name), rviz_config_dir_name, rviz_config_file_name])
    rviz_config_dir_arg = DeclareLaunchArgument('d', default_value=rviz_config_dir)
    rviz_config_dir_f = LaunchConfiguration('d')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir_f])

    
    return LaunchDescription([
        map_file_arg,
        LogInfo(msg=["Map file path: ", map_file_path]),
        map_server_node,
        lifecycle_manager_node,
        rviz_config_dir_arg,
        rviz_node
    ])