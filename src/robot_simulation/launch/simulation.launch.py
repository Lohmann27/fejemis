import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_simulation_package = 'robot_simulation'
    robot_description_package = 'robot_description'
    robot_control_package = 'robot_control'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_description_package),'launch','robot_xacro.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_simulation_package),'launch','rviz.launch.py'
        )]), launch_arguments={'use_sim_time': 'true','view_laserscan': 'false','view_robot': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_simulation_package),'launch','gz.launch.py'
        )]))


    # Launch the control launch file launching both keyboard and controller in the same launch file. 
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_control_package),'launch','control.launch.py'
        )]),launch_arguments={'use_sim_time': 'true','use_joy': 'true','use_key': 'true'}.items()   
    )

    
    return LaunchDescription([
        rsp,
        gazebo,
        rviz,
        control

    ])
