from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    # Launch global_costmap node
    Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[
            # Load global costmap parameters from a YAML file
            os.path.join(get_package_share_directory('your_package_name'), 'config', 'global_costmap_params.yaml')
        ]
    ),

    # Launch local_costmap node
    Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[
            # Load local costmap parameters from a YAML file
            os.path.join(get_package_share_directory('your_package_name'), 'config', 'local_costmap_params.yaml')
        ]
    ),
  ])
