from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bridge',
            #namespace='turtlesim1',
            executable='hw_bridge',
            name='bridge',
            #remappings=[
                #('/input/pose', '/turtlesim1/turtle1/pose'),
                #('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            #]
            arguments=['-d']
        )
    ])
