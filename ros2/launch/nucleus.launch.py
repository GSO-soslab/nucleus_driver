import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('nucleus_driver_ros2'),
        'config',
        'nucleus_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='nucleus_driver_ros2',
            executable='nucleus_node',
            name='nucleus_node',
            output='screen',
            parameters=[config_file]
        )
    ])