#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'map_server'
    map_file_name = 'warehouse_map_sim.yaml'
    map_file_path = os.path.join(get_package_share_directory(pkg_name), 'config', map_file_name)

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
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_name), 'rviz', rviz_config_file_name)
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
        map_server_node,
        lifecycle_manager_node,
        rviz_config_dir_arg,
        rviz_node
    ])