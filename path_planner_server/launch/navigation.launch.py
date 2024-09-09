#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = 'path_planner_server'
    pkg_share_name = FindPackageShare(pkg_name)
    config_dir_name = 'config'

    # ========
    # ARGUMENT
    # ========
    # use_sim_time argument (determines simulator vs real lab)
    use_sim_time_f = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value=TextSubstitution(text='true'),
            description='config for simulator (true) or lab (false)')
    
    # ====
    # NODE
    # ====
    # `base_link` broadcaster node
    static_tf_base_link_node = Node(
            package='path_planner_server', executable='static_send_tf_base_link', output='screen',
            name='static_send_tf_base_link', parameters=[{'use_sim_time': use_sim_time_f}])

    # ======
    # CONFIG
    # ======
    # Rviz2 config file path argument
    rviz_config_dir_name = 'rviz'

    rviz_config_file_name_sim = 'nav_config_sim.rviz'
    rviz_config_dir_sim = PathJoinSubstitution([pkg_share_name, rviz_config_dir_name, rviz_config_file_name_sim])

    rviz_config_file_name_lab = 'nav_config_lab.rviz'
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
            default_value=TextSubstitution(text=default_map_file_name),
            description='simulator or real lab map file name')
    map_file_path = PathJoinSubstitution([FindPackageShare(map_svr_pkg_name), map_dir_name, map_file_f])

    # ====
    # NODE
    # ====
    # map server node
    map_server_node = Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, {'yaml_filename': map_file_path}])

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
    # planner node
    planner_config_file_name = 'planner_server.yaml'
    nav2_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, planner_config_file_name])
    planner_node = Node(
            package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, nav2_yaml])

    # ======
    # CONFIG
    # ======
    # controller node configuration
    controller_config_file_name_sim = 'controller_sim.yaml'
    controller_yaml_sim = PathJoinSubstitution([pkg_share_name, config_dir_name, controller_config_file_name_sim])

    controller_config_file_name_lab = 'controller_lab.yaml'
    controller_yaml_lab = PathJoinSubstitution([pkg_share_name, config_dir_name, controller_config_file_name_lab])

    # ====
    # NODE
    # ====
    # controller node (configured for sim or lab)
    controller_node_sim = Node(
            package='nav2_controller', executable='controller_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, controller_yaml_sim],
            remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')],
            condition=IfCondition(use_sim_time_f))

    controller_node_lab = Node(
            package='nav2_controller', executable='controller_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, controller_yaml_lab],
            condition=UnlessCondition(use_sim_time_f))

    # ====
    # NODE
    # ====
    # manager of behavior behaviors node
    behavior_config_file_name = 'behavior.yaml'
    behavior_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, behavior_config_file_name])
    behavior_svr_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[{'use_sim_time': use_sim_time_f}, behavior_yaml],
            output='screen')

    # ====
    # NODE
    # ====
    # behavior tree navigator node
    bt_nav_config_file_name = 'bt_navigator.yaml'
    bt_navigator_yaml = PathJoinSubstitution([pkg_share_name, config_dir_name, bt_nav_config_file_name])
    bt_nav_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f}, bt_navigator_yaml])
            
    # ====
    # NODE
    # ====
    # lifecycle manager node (note: unique name)
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_f},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server', 
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}])
    
    # ==================
    # LAUNCH DESCRIPTION
    # ==================
    return LaunchDescription([
        use_sim_time_arg,
        static_tf_base_link_node,
        rviz_node_sim,
        rviz_node_lab,
        map_file_arg,
        map_server_node,
        amcl_node_sim,
        amcl_node_lab,
        planner_node,
        controller_node_sim,
        controller_node_lab,
        behavior_svr_node,
        bt_nav_node,
        lifecycle_manager_node
    ])
