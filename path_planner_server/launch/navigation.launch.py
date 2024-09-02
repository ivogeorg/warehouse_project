#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # SIMULATOR <======OR=====> REAL ROBOT LAB
    use_sim_time = True
    
    pkg_name = 'path_planner_server'
    pkg_share_name = FindPackageShare(pkg_name)
    config_dir_name = 'config'

    # `base_link` broadcaster node
    static_tf_base_link_node = Node(
            package='path_planner_server',
            executable='static_send_tf_base_link',
            output='screen',
            name='static_send_tf_base_link',
            parameters=[{'use_sim_time': use_sim_time}])

    # Rviz2 config file path argument
    rviz_config_file_name = 'paconfig.rviz'
    rviz_config_dir_name = 'rviz'
    rviz_config_dir = PathJoinSubstitution([pkg_share_name, rviz_config_dir_name, rviz_config_file_name])
    rviz_config_dir_arg = DeclareLaunchArgument('d', default_value=rviz_config_dir)
    rviz_config_dir_f = LaunchConfiguration('d')

    # Rviz2 node
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_dir_f])

    # map server node
    map_svr_pkg_name = 'map_server'
    default_map_file_name = 'warehouse_map_sim.yaml'

    # Capture command line or launch file argument 'map_file'
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
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_file_path}])

    # localization node (amcl)
    amcl_config_file_name = 'amcl_config.yaml'
    amcl_config_dir_name = 'config'
    amcl_config_file_path = PathJoinSubstitution([pkg_share_name, amcl_config_dir_name, amcl_config_file_name])
    amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, amcl_config_file_path])

    # planner node
    planner_config_file_name = 'planner_server.yaml'
    nav2_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, planner_config_file_name])
    planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, nav2_yaml])

    # controller node
    controller_config_file_name = 'controller.yaml'
    controller_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, controller_config_file_name])
    controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, controller_yaml],
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')])

    # manager of behavior behaviors node
    behavior_config_file_name = 'behavior.yaml'
    behavior_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, behavior_config_file_name])
    behavior_svr_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[{'use_sim_time': use_sim_time}, behavior_yaml],
            output='screen')

    # behavior tree navigator node
    bt_nav_config_file_name = 'bt_navigator.yaml'
    bt_navigator_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, bt_nav_config_file_name])
    bt_nav_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, bt_navigator_yaml])
            
    # lifecycle manager node (note: unique name)
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server', 
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
    )
    
    return LaunchDescription([
        static_tf_base_link_node,
        rviz_config_dir_arg,
        rviz_node,
        map_file_arg,
        map_server_node,
        amcl_node,
        planner_node,
        controller_node,
        behavior_svr_node,
        bt_nav_node,
        lifecycle_manager_node
    ])
