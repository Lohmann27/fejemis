from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # choose which cmd_vel_path to use
    cmd_vel_path = LaunchConfiguration('cmd_vel_path', default='/cmd_vel_joy')

    package_name = 'robot_control'

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )
    
    teleop_node = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name = 'teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', cmd_vel_path)]
            )

    return LaunchDescription([
        joy_node,
        teleop_node       
    ])