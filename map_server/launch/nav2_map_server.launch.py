#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_file_name = 'warehouse_map_sim.yaml'
    map_file_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file_name)

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path} 
                       ]
    )

    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
    )

    # TODO: add rviz2_node
    
    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node
    ])