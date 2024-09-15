# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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
    
    

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_simulation_package = get_package_share_directory('robot_simulation')
    

    # Path to the world file
    world_file_path = os.path.join(
        robot_simulation_package,
        'worlds',
        'lidar_world.sdf'
    )

    # Ensure the world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

    # Launch Gazebo with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items()
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'gpu_lidar_bridge.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz,
    ])