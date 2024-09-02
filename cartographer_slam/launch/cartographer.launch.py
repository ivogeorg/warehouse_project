#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'cartographer_slam'
    pkg_share_path = FindPackageShare(pkg_name)
    config_dir_name = 'config'

    cartographer_config_file_name = 'cartographer_lab.lua'

    # environment_arg = DeclareLaunchArgument('environ', default_value='lab')
    # environment_f = LaunchConfiguration('environ')
    # if (environment_f == 'lab'):
    #     cartographer_config_file_name = 'cartographer_lab.lua'
    # else:
    #     cartographer_config_file_name = 'cartographer_sim.lua'


    # Rviz2 config file path argument
    rviz_config_file_name = 'caconfig.rviz'
    rviz_config_dir_name = 'rviz'
    rviz_config_dir = PathJoinSubstitution([pkg_share_path, rviz_config_dir_name, rviz_config_file_name])
    rviz_config_dir_arg = DeclareLaunchArgument('d', default_value=rviz_config_dir)
    rviz_config_dir_f = LaunchConfiguration('d')

    # Rviz2 node
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir_f])

    # cartographer node (SLAM)
    config_dir_arg = DeclareLaunchArgument("configuration_directory", default_value=PathJoinSubstitution([pkg_share_path, config_dir_name]))
    config_basename_arg = DeclareLaunchArgument("configuration_basename", default_value=TextSubstitution(text=cartographer_config_file_name))

    cartographer_config_dir = LaunchConfiguration("configuration_directory")
    configuration_basename = LaunchConfiguration("configuration_basename")

    cartographer_node = Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
    )

    # occupancy grid node
    resolution_arg = DeclareLaunchArgument('resolution', default_value='0.05')
    publish_period_arg = DeclareLaunchArgument('publish_period_sec', default_value='1.0')

    resolution_f = LaunchConfiguration('resolution')
    publish_period_f = LaunchConfiguration('publish_period_sec')

    ocupancy_grid_node = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node', # from Discord
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', resolution_f, 
                       '-publish_period_sec', publish_period_f]    
    )
    
    return LaunchDescription([
        # environment_arg,
        rviz_config_dir_arg,
        rviz_node,
        LogInfo(msg=["Rviz2 config file path: ", rviz_config_dir_f]),
        # LogInfo(msg=["Environment: ", environment_f]),
        LogInfo(msg=["Lua file: ", cartographer_config_file_name]),
        config_dir_arg,
        config_basename_arg,
        cartographer_node,
        resolution_arg,
        publish_period_arg,
        ocupancy_grid_node
    ])