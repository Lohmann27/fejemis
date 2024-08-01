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

    package_name = 'robot_simulation'

   # Path to the package where the included launch file is located
    robot_description_path = 'robot_description'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(robot_description_path),'launch','robot_xacro.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'all_in_one.rviz')]
        )


    
    return LaunchDescription([
        rsp,
        rviz
    ])
