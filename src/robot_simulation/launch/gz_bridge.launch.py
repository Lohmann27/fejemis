import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the configuration file
    config_file_path = os.path.join(
        get_package_share_directory('robot_simulation'),
        'config',
        'gz_bridge.yaml'
    )

    # Node to run the parameter bridge
    parameter_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': config_file_path}],
        output='screen'
    )

    return LaunchDescription([
        parameter_bridge_node
    ])
