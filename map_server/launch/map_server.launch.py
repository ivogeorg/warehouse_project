#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'map_server'

    use_sim_time_f = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=TextSubstitution(text='True'))
    
    map_dir_name = 'maps'
    default_map_file_name = 'warehouse_map_sim.yaml'

    # map server node
    # Capture command line or launch file argument 'map_file'
    #
    # Note: 
    # This is only the file name and not the full path, so
    # construct the path and then pass to the 'yaml_filename'
    # parameter of the 'map_server' executable.
    map_file_f = LaunchConfiguration('map_file')
    map_file_arg = DeclareLaunchArgument(
                        'map_file', 
                        default_value=TextSubstitution(text=default_map_file_name))
    map_file_path = PathJoinSubstitution([FindPackageShare(pkg_name), map_dir_name, map_file_f])

    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, 
                        {'yaml_filename':map_file_path}])

    # lifecycle manager node
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    # rviz node
    rviz_config_file_name = 'map_server_config.rviz'
    rviz_config_dir_name = 'rviz'
    rviz_config_dir = PathJoinSubstitution([FindPackageShare(pkg_name), rviz_config_dir_name, rviz_config_file_name])
    rviz_config_dir_f = LaunchConfiguration('d')
    rviz_config_dir_arg = DeclareLaunchArgument('d', default_value=rviz_config_dir)

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time_f}],
            arguments=['-d', rviz_config_dir_f])

    
    return LaunchDescription([
        use_sim_time_arg,        
        rviz_config_dir_arg,
        rviz_node,
        map_file_arg,
        LogInfo(msg=["Map file path: ", map_file_path]),
        LogInfo(msg=["use_sim_time: ", use_sim_time_f]),
        map_server_node,
        lifecycle_manager_node,
    ])