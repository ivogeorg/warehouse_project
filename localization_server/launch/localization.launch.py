#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'localization_server'
    pkg_share_name = FindPackageShare(pkg_name)

    # ========
    # ARGUMENT
    # ========
    # use_sim_time argument (determines simulator vs real lab)
    use_sim_time_f = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=TextSubstitution(text='true'))
    
    # ======
    # CONFIG
    # ======
    # Rviz2 config file path argument
    rviz_config_dir_name = 'rviz'

    rviz_config_file_name_sim = 'localization_config_sim.rviz'
    rviz_config_dir_sim = PathJoinSubstitution([pkg_share_name, rviz_config_dir_name, rviz_config_file_name_sim])

    rviz_config_file_name_lab = 'localization_config_lab.rviz'
    rviz_config_dir_lab = PathJoinSubstitution([pkg_share_name, rviz_config_dir_name, rviz_config_file_name_lab])

    # ====
    # NODE
    # ====
    # Rviz2 node (configured for sim or lab)
    rviz_node_sim = Node(
                        package='rviz2', executable='rviz2', output='screen', name='rviz_node',
                        parameters=[{'use_sim_time': use_sim_time_f}],
                        arguments=['-d', rviz_config_dir_sim],
                        condition=IfCondition(use_sim_time_f))

    rviz_node_lab = Node(
                        package='rviz2', executable='rviz2', output='screen', name='rviz_node',
                        parameters=[{'use_sim_time': use_sim_time_f}],
                        arguments=['-d', rviz_config_dir_lab],
                        condition=UnlessCondition(use_sim_time_f))

    # ======
    # CONFIG
    # ======
    # map configuration
    map_svr_pkg_name = 'map_server'
    map_dir_name = 'maps'
    default_map_file_name = 'warehouse_map_sim.yaml'

    # ========
    # ARGUMENT
    # ========
    # map_file argument (maps are accessed from map_server/maps)
    map_file_f = LaunchConfiguration('map_file')
    map_file_arg = DeclareLaunchArgument(
                        'map_file', 
                        default_value=TextSubstitution(text=default_map_file_name))
    map_file_path = PathJoinSubstitution([FindPackageShare(map_svr_pkg_name), map_dir_name, map_file_f])

    # ====
    # NODE
    # ====
    # map server node
    map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, 
                        {'yaml_filename': map_file_path}])

    # ======
    # CONFIG
    # ======
    # amcl configuration (amcl nodes are inline and mutually exclusive)
    amcl_config_dir_name = 'config'

    amcl_config_file_name_sim = 'amcl_config_sim.yaml'
    amcl_config_file_path_sim = PathJoinSubstitution([pkg_share_name, amcl_config_dir_name, amcl_config_file_name_sim])

    amcl_config_file_name_lab = 'amcl_config_lab.yaml'
    amcl_config_file_path_lab = PathJoinSubstitution([pkg_share_name, amcl_config_dir_name, amcl_config_file_name_lab])

    # ====
    # NODE
    # ====
    # amcl node (configured for sim or lab)
    amcl_node_sim = Node(
                        package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
                        parameters=[{'use_sim_time': use_sim_time_f}, amcl_config_file_path_sim],
                        condition=IfCondition(use_sim_time_f))

    amcl_node_lab = Node(
                        package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
                        parameters=[{'use_sim_time': use_sim_time_f}, amcl_config_file_path_lab],
                        condition=UnlessCondition(use_sim_time_f))

    # ====
    # NODE
    # ====
    # lifecycle manager node
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
    )
    
    # ==================
    # LAUNCH DESCRIPTION
    # ==================
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        rviz_node_sim,
        rviz_node_lab,
        map_server_node,
        amcl_node_sim,
        amcl_node_lab,
        lifecycle_manager_node
    ])