import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():

    # rviz views
    view_laserscan = LaunchConfiguration('view_laserscan', default='true')
    view_robot = LaunchConfiguration('view_robot', default='true')

    package_name = 'robot_simulation'

    # Conditional launch for laserscan RViz view
    rviz_laser = GroupAction([
            LogInfo(condition=IfCondition(view_laserscan), msg="Launching RViz1"),
            Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_laser',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'laserscan.rviz')]
        ),
    ], condition=IfCondition(view_laserscan))
 
    # Conditional launch for robot RViz view
    rviz_robot = GroupAction([
            LogInfo(condition=IfCondition(view_robot), msg="Launching RViz2"),
            Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'view_robot.rviz')]
        ),
    ], condition=IfCondition(view_robot))
 
    return LaunchDescription([
        rviz_laser,
        rviz_robot
    ])
