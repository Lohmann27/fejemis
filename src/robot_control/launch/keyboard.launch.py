import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Check if we're told to use sim time
    cmd_vel_path = LaunchConfiguration('cmd_vel_path', default='/cmd_vel_key')

    # Define the package names
    robot_control_package = 'robot_control'

    # Define the teleop_twist_keyboard node
    keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # This will open teleop in a new terminal window
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/cmd_vel', cmd_vel_path)  # Remap to the appropriate topic
        ]
    )

    # Return the LaunchDescription containing both the included launch file and the keyboard node
    return LaunchDescription([
        keyboard
    ])
