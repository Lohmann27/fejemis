from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    package_name = 'robot_control'

    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'diff_drive_controller.yaml'  
    )
    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file_path],
            output='screen',
        )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        controller_manager,
        diff_drive_spawner,
        joint_broad_spawner
    ])