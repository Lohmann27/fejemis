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


    

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_simulation_package),'launch','gz.launch.py'
        )]))


    # Launch the keyboard launch file
    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_control_package),'launch','keyboard.launch.py'
        )]), launch_arguments={'cmd_vel_path': '/cmd_vel_key'}.items() # the same path as the yaml file.
    )

    # # # Launch the keyboard launch file
    # # joystick = IncludeLaunchDescription(
    # #     PythonLaunchDescriptionSource([os.path.join(
    # #         get_package_share_directory(robot_control_package),'launch','joystick.launch.py'
    # #     )]), launch_arguments={'cmd_vel_path': '/cmd_vel_joy'}.items() # the same path as the yaml file.
    # # )

    # Launch the control launch file launching both keyboard and controller in the same launch file. 
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_control_package),'launch','control.launch.py'
        )]))

    
    return LaunchDescription([
        gazebo,
        #keyboard,
        #joystick,
        control
    ])
