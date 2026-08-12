import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Path to the default YAML configuration file
    config_file = os.path.join(
        get_package_share_directory('nucleus_driver_ros2'),
        'config',
        'nucleus_config.yaml'
    )

    # # Declare the 'namespace' launch argument
    # namespace_launch_arg = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='nucleus_node',
    #     description='Namespace for the Nucleus node'
    # )

    # # Declare the 'frame_id' launch argument
    # frame_id_launch_arg = DeclareLaunchArgument(
    #     'frame_id',
    #     default_value='nucleus_dvl',
    #     description='The TF frame ID for the Nucleus sensor'
    # )
    
    # # Use LaunchConfiguration to get the values
    # namespace = LaunchConfiguration('namespace')
    # frame_id = LaunchConfiguration('frame_id')

    nucleus_node = Node(
        package='nucleus_driver_ros2',
        executable='nucleus_node',
        name='nucleus_node',
        namespace='alpha_rise',
        output='screen',
        # Load parameters from the YAML file first...
        parameters=[
            config_file,
        ]
    )


    return LaunchDescription([
        nucleus_node
    ])
