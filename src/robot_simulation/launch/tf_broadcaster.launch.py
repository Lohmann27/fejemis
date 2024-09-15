from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Transform broadcaster node
        Node(
            package='robot_simulation',
            executable='tf_broadcaster.py',  # Script name only, no path
            name='transform_broadcaster',
            output='screen',
        ),
    ])
