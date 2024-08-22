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
        rviz_config_dir_arg,
        rviz_node
    ])