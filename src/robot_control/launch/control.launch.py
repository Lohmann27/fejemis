from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Use different controllers
    use_joy = LaunchConfiguration('use_joy')
    use_key = LaunchConfiguration('use_key')


    package_name = 'robot_control'

    # Conditional launch for # joystick input to send control parameters
    joystick = GroupAction([
            LogInfo(condition=IfCondition(use_joy), msg="Using Joystick"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join( # type: ignore
                        get_package_share_directory(package_name),'launch','joystick.launch.py'
                    )]), launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
    ], condition=IfCondition(use_joy))
 
    # Conditional launch for # keyboard (PS5 or other) input to send control parameters
    keyboard = GroupAction([
            LogInfo(condition=IfCondition(use_key), msg="Using Keyboard"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join( # type: ignore
                        get_package_share_directory(package_name),'launch','keyboard.launch.py'
                    )]), launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
    ], condition=IfCondition(use_key))


    #For controlling priority for wich control should be used first
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( # type: ignore
           get_package_share_directory(package_name),'launch','twist_mux.launch.py'
        )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Controlling diff_drive_spawner and joint_broad_spawner
    controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( # type: ignore
            get_package_share_directory(package_name),'launch','controller_manager.launch.py'
        )])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_joy',
            default_value='false',
            description='Use keyboard if true'),
        DeclareLaunchArgument(
            'use_key',
            default_value='true',
            description='Use joystick if true'),
        joystick,
        keyboard,
        twist_mux,
        controller_manager,
    ])