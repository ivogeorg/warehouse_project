#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'localization_server'

    map_file_name = 'warehouse_map_sim.yaml'
    map_file_arg = DeclareLaunchArgument('map_file', default_value=map_file_name)
    map_file_f = LaunchConfiguration('map_file')

    # TODO: Get the map file from the package that generated it
    map_file_name = 'warehouse_map_sim.yaml'
    map_file_path = os.path.join(get_package_share_directory(pkg_name), 'config', map_file_name)

    # TODO: Only the filename is passed from the command line but the parameter should
    #       be the full path name. Create an argument the value of which will be used
    #       to construct the expected parameter value.
    # NOTE: This will require a substitution!!!
    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path}])

    amcl_config_file_name = 'amcl_config.yaml'
    amcl_config_file_path = os.path.join(get_package_share_directory(pkg_name), 'config', amcl_config_file_name)
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config_file_path])

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
    )
    
    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])